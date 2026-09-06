#!/usr/bin/env python3
"""
Post-process an ACDC4Robot (Fusion 360) URDF export so it loads cleanly in
viewers / MuJoCo / Isaac Sim and is usable for sim2real.

    python3 fix_acdc_export.py robot/robot.urdf [options]

Steps (always reads the raw export, so it is safe to re-run):
  1. removes duplicated joints (same name)                  -> single-parent tree
  2. decimates STL meshes above MuJoCo's 200k-face limit   -> meshes/*.stl in place
  3a. re-expresses the ROOT link inertial in the visual frame. ACDC4Robot writes
      the root link's visual relative to the Fusion world frame but its inertial
      relative to the component frame, so root CoM/inertia come out rotated.
  3.  optional --rotate-links: rotates visual/collision/inertial of the named
      links in place (use when a Fusion component frame is rotated vs. the robot)
  3b. collision: coacd (default) = convex-decomposition pieces per link that
      follow the mesh (good for self-collision); primitive = one fitted
      box/cylinder/sphere per link; keep = leave the exported mesh collision
  4.  validates: single tree, no missing links, inertia positive definite and
      I1+I2>=I3, left/right principal-moment symmetry; loads in MuJoCo and
      renders <name>_fixed.png (visual / collision / overlay rows)

Output: <name>_fixed.urdf next to the input (input file is left untouched).
Dependencies: numpy scipy trimesh fast-simplification coacd mujoco pillow
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

FACE_LIMIT = 200_000
DECIMATE_TO = 120_000


def rpy_to_R(r, p, y):
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    # URDF rpy is extrinsic x-y-z (R = Rz(y) Ry(p) Rx(r)); scipy handles the
    # pitch = +-90 deg gimbal-lock case that a naive asin/atan2 gets wrong.
    from scipy.spatial.transform import Rotation
    return tuple(Rotation.from_matrix(np.asarray(R, float)).as_euler("xyz"))


def dedupe_joints(root):
    seen, removed = set(), 0
    for j in list(root.findall("joint")):
        if j.get("name") in seen:
            root.remove(j)
            removed += 1
        seen.add(j.get("name"))
    return removed


def _rotate_origin(e, Rfix):
    xyz = np.array([float(v) for v in e.get("xyz", "0 0 0").split()])
    r, p, y = [float(v) for v in e.get("rpy", "0 0 0").split()]
    e.set("xyz", "%.9g %.9g %.9g" % tuple(Rfix @ xyz))
    e.set("rpy", "%.9g %.9g %.9g" % R_to_rpy(Rfix @ rpy_to_R(r, p, y)))


def rotate_link_contents(link, Rfix):
    for tag in ("visual", "collision"):
        for geom in link.findall(tag):
            e = geom.find("origin")
            if e is None:
                e = ET.SubElement(geom, "origin")
                e.set("xyz", "0 0 0")
                e.set("rpy", "0 0 0")
            _rotate_origin(e, Rfix)

    # Inertia: fold the rotation into the tensor (I' = R I R^T) and keep the
    # inertial frame rpy at 0. Some viewers ignore the inertial origin's rpy,
    # so a rotated frame would draw the inertia box misaligned with the mesh.
    o = link.find("inertial/origin")
    it = link.find("inertial/inertia")
    if o is not None and it is not None:
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        r, p, y = [float(v) for v in o.get("rpy", "0 0 0").split()]
        R = Rfix @ rpy_to_R(r, p, y)
        ixx, iyy, izz = (float(it.get(k)) for k in ("ixx", "iyy", "izz"))
        ixy, ixz, iyz = (float(it.get(k, "0")) for k in ("ixy", "ixz", "iyz"))
        I = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        In = R @ I @ R.T
        o.set("xyz", "%.9g %.9g %.9g" % tuple(Rfix @ xyz))
        o.set("rpy", "0 0 0")
        for k, v in (("ixx", In[0, 0]), ("iyy", In[1, 1]), ("izz", In[2, 2]),
                     ("ixy", In[0, 1]), ("ixz", In[0, 2]), ("iyz", In[1, 2])):
            it.set(k, "%.9g" % v)


def _origin_T(e):
    xyz = np.array([float(v) for v in e.get("xyz", "0 0 0").split()])
    r, p, y = [float(v) for v in e.get("rpy", "0 0 0").split()]
    return rpy_to_R(r, p, y), xyz


def fix_root_inertial(root):
    """ACDC4Robot writes the root link's visual relative to the Fusion world
    frame but its inertial relative to the component's own frame, so the
    root link's CoM/inertia come out rotated by the component pose. Re-express
    the inertial in the same frame as the visual (visual origin = that pose)."""
    links = {l.get("name"): l for l in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        return None
    link = links[roots[0]]
    vis = link.find("visual/origin")
    o = link.find("inertial/origin")
    it = link.find("inertial/inertia")
    if vis is None or o is None or it is None:
        return None
    R, t = _origin_T(vis)
    Ro, com = _origin_T(o)
    if np.abs(Ro - np.eye(3)).max() > 1e-6:
        return None  # not the plain ACDC layout (CoM frame aligned with link) - leave alone
    g = lambda k: float(it.get(k, "0"))
    I = np.array([[g("ixx"), g("ixy"), g("ixz")],
                  [g("ixy"), g("iyy"), g("iyz")],
                  [g("ixz"), g("iyz"), g("izz")]])
    In = R @ I @ R.T
    o.set("xyz", "%.9g %.9g %.9g" % tuple(R @ com + t))
    for k, v in (("ixx", In[0, 0]), ("iyy", In[1, 1]), ("izz", In[2, 2]),
                 ("ixy", In[0, 1]), ("ixz", In[0, 2]), ("iyz", In[1, 2])):
        it.set(k, "%.9g" % v)
    return roots[0]


def rotate_root_frame(root, Rfix):
    """Rotate the whole robot by rotating the root link's contents and the
    origins of every joint attached to the root link. Used to bring the
    robot to the +X-forward / +Y-left / +Z-up convention that locomotion RL
    frameworks (Isaac Lab, MJX) assume for velocity commands and rewards."""
    links = {l.get("name"): l for l in root.findall("link")}
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = [n for n in links if n not in children]
    if len(roots) != 1:
        return None
    rotate_link_contents(links[roots[0]], Rfix)
    n = 0
    for j in root.findall("joint"):
        if j.find("parent").get("link") != roots[0]:
            continue
        o = j.find("origin")
        if o is None:
            o = ET.SubElement(j, "origin")
            o.set("xyz", "0 0 0")
            o.set("rpy", "0 0 0")
        _rotate_origin(o, Rfix)   # joint <axis> is in the joint frame: unchanged
        n += 1
    return roots[0], n


def decimate_meshes(mesh_dir):
    import trimesh

    for f in sorted(os.listdir(mesh_dir)):
        if not f.lower().endswith(".stl"):
            continue
        path = os.path.join(mesh_dir, f)
        mesh = trimesh.load(path)
        n = len(mesh.faces)
        if n > FACE_LIMIT:
            dec = mesh.simplify_quadric_decimation(face_count=DECIMATE_TO)
            dec.export(path)
            print("  decimated %-36s %d -> %d faces" % (f, n, len(dec.faces)))


def _circularity(mesh, Tb, axis):
    """Spread of convex-hull radii of the cross-section perpendicular to `axis`
    (box-frame axis index). ~0 for a circle, >0.12 for square-ish sections."""
    from scipy.spatial import ConvexHull

    local = (np.linalg.inv(Tb) @ np.c_[mesh.vertices, np.ones(len(mesh.vertices))].T).T[:, :3]
    plane = np.delete(local, axis, axis=1)
    hull = ConvexHull(plane)
    r = np.linalg.norm(plane[hull.vertices], axis=1)
    return float(r.std() / r.mean())


def _classify(mesh, ext, Tb, joint_axis_box):
    """Port of the Rex fusion2URDF fit_primitive rules: dimension pattern,
    volume fill ratio, revolute-joint hint, with a circular-cross-section
    check before ever choosing a cylinder. Returns (shape, axis_index)."""
    dims = sorted((float(e), i) for i, e in enumerate(ext))
    (d0, a0), (d1, a1), (d2, a2) = dims
    r01, r12, r02 = d0 / d1, d1 / d2, d0 / d2
    fill = abs(mesh.volume) / float(np.prod(ext))
    circular = lambda ax: _circularity(mesh, Tb, ax) < 0.12

    if r02 > 0.80:                                   # near-cube
        if 0.40 <= fill < 0.65:
            return "sphere", None
        return "box", None
    if r01 < 0.15 and r12 > 0.6:                     # plate
        return "box", None
    if r01 < 0.5 and r12 > 0.85 and fill < 0.88:     # disc
        return ("cylinder", a0) if circular(a0) else ("box", None)
    if r12 < 0.35 and r01 > 0.8 and fill < 0.88:     # rod with round section
        return ("cylinder", a2) if circular(a2) else ("box", None)
    if joint_axis_box is not None and circular(joint_axis_box):   # revolute hint
        return "cylinder", joint_axis_box
    return "box", None


def primitive_collisions(root, mesh_dir):
    """Replace each link's mesh collision with one fitted primitive
    (box / cylinder / sphere) computed from the visual mesh in the link frame."""
    import trimesh

    parent_joint = {j.find("child").get("link"): j for j in root.findall("joint")}
    out = []
    for link in root.findall("link"):
        vis = link.find("visual")
        if vis is None or vis.find("geometry/mesh") is None:
            continue
        mesh_file = vis.find("geometry/mesh").get("filename")
        scale = float(vis.find("geometry/mesh").get("scale", "1 1 1").split()[0])
        R, t = _origin_T(vis.find("origin"))
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, t
        mesh = trimesh.load(os.path.join(os.path.dirname(mesh_dir), mesh_file))
        mesh.apply_scale(scale)
        mesh.apply_transform(T)
        # Prefer the axis-aligned box (in the link frame) unless the oriented
        # box is clearly tighter: a minimum-volume OBB happily picks a diagonal
        # orientation for boxy parts like the torso, which is worse for contact.
        aabb = mesh.bounding_box
        obb = mesh.bounding_box_oriented
        if aabb.volume <= 1.3 * obb.volume:
            ext, Tb = aabb.primitive.extents, aabb.primitive.transform
        else:
            ext, Tb = obb.primitive.extents, obb.primitive.transform
        ext = np.asarray(ext, float)

        # joint axis (link frame) expressed in the box frame -> dominant box axis
        j = parent_joint.get(link.get("name"))
        joint_axis_box = None
        if j is not None and j.get("type") in ("revolute", "continuous") and j.find("axis") is not None:
            ax = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            joint_axis_box = int(np.argmax(np.abs(Tb[:3, :3].T @ ax)))

        shape, axis = _classify(mesh, ext, Tb, joint_axis_box)

        for c in link.findall("collision"):
            link.remove(c)
        col = ET.SubElement(link, "collision")
        col.set("name", link.get("name") + "_collision")
        o = ET.SubElement(col, "origin")
        geo = ET.SubElement(col, "geometry")
        Rc = Tb[:3, :3]
        if shape == "box":
            ET.SubElement(geo, "box").set("size", "%.5g %.5g %.5g" % tuple(ext))
            desc = "box %s" % (tuple(round(float(e) * 1000) for e in ext),)
        elif shape == "sphere":
            rad = float(ext.sum() / 6.0)
            ET.SubElement(geo, "sphere").set("radius", "%.5g" % rad)
            desc = "sphere r=%d" % round(rad * 1000)
        else:
            # URDF cylinders are along local Z: rotate the box frame so its
            # `axis` column becomes Z (keep a right-handed frame).
            perm = {0: [1, 2, 0], 1: [2, 0, 1], 2: [0, 1, 2]}[axis]
            Rc = Rc[:, perm]
            length = float(ext[axis])
            rad = float(max(np.delete(ext, axis)) / 2.0)
            cyl = ET.SubElement(geo, "cylinder")
            cyl.set("radius", "%.5g" % rad)
            cyl.set("length", "%.5g" % length)
            desc = "cylinder r=%d L=%d" % (round(rad * 1000), round(length * 1000))
        o.set("xyz", "%.6g %.6g %.6g" % tuple(Tb[:3, 3]))
        o.set("rpy", "%.6g %.6g %.6g" % R_to_rpy(Rc))
        out.append((link.get("name"), desc))
    return out


COACD_THRESHOLD = 0.05      # concavity tolerance (lower = tighter, more pieces)
COACD_MAX_HULLS = 12
COACD_MAX_HULL_VERTS = 64   # keep each piece cheap for the physics engine


def convex_decomposition_collisions(root, mesh_dir, threshold=COACD_THRESHOLD,
                                    max_hulls=COACD_MAX_HULLS, max_verts=COACD_MAX_HULL_VERTS):
    """Replace each link's collision with a set of convex pieces (CoACD) that
    follow the visual mesh closely, including concave regions. Pieces are
    written to meshes/collision/<link>_<i>.stl and referenced with the same
    origin/scale as the visual mesh."""
    import coacd
    import trimesh

    coacd.set_log_level("off")
    col_dir = os.path.join(mesh_dir, "collision")
    os.makedirs(col_dir, exist_ok=True)
    out = []
    for link in root.findall("link"):
        vis = link.find("visual")
        if vis is None or vis.find("geometry/mesh") is None:
            continue
        mesh_el = vis.find("geometry/mesh")
        mesh = trimesh.load(os.path.join(os.path.dirname(mesh_dir), mesh_el.get("filename")))
        parts = coacd.run_coacd(
            coacd.Mesh(mesh.vertices, mesh.faces),
            threshold=threshold, max_convex_hull=max_hulls,
            # max_ch_vertex is only enforced when decimate=True
            decimate=True, max_ch_vertex=max_verts, merge=True, seed=0)

        for c in link.findall("collision"):
            link.remove(c)
        origin = vis.find("origin")
        n_faces = 0
        for i, (v, f) in enumerate(parts):
            piece = trimesh.Trimesh(v, f)
            rel = "meshes/collision/%s_%d.stl" % (link.get("name"), i)
            piece.export(os.path.join(os.path.dirname(mesh_dir), rel))
            n_faces += len(f)
            col = ET.SubElement(link, "collision")
            col.set("name", "%s_collision_%d" % (link.get("name"), i))
            o = ET.SubElement(col, "origin")
            o.set("xyz", origin.get("xyz"))
            o.set("rpy", origin.get("rpy"))
            m = ET.SubElement(ET.SubElement(col, "geometry"), "mesh")
            m.set("filename", rel)
            m.set("scale", mesh_el.get("scale", "1 1 1"))
        out.append((link.get("name"), "%d convex pieces, %d faces" % (len(parts), n_faces)))
        print("    %-32s %s" % out[-1], flush=True)
    return out


def validate(root):
    links = {l.get("name") for l in root.findall("link")}
    joints = root.findall("joint")
    children = [j.find("child").get("link") for j in joints]
    refs = set(children) | {j.find("parent").get("link") for j in joints}
    problems = []
    if refs - links:
        problems.append("joints reference missing links: %s" % sorted(refs - links))
    dup = {c for c in children if children.count(c) > 1}
    if dup:
        problems.append("links with multiple parents: %s" % sorted(dup))
    roots = links - set(children)
    if len(roots) != 1:
        problems.append("expected exactly 1 root link, got %s" % sorted(roots))

    # Inertia checks (from Rex/urdf_render/check_inertia.py): every tensor must
    # be positive definite with I1 + I2 >= I3, and mirrored links must have
    # matching principal moments — mass alone cannot reveal a misplaced member.
    principal = {}
    for l in root.findall("link"):
        it = l.find("inertial/inertia")
        if it is None:
            continue
        g = lambda k: float(it.get(k, "0"))
        I = np.array([[g("ixx"), g("ixy"), g("ixz")],
                      [g("ixy"), g("iyy"), g("iyz")],
                      [g("ixz"), g("iyz"), g("izz")]])
        ev = np.sort(np.linalg.eigvalsh(I))
        principal[l.get("name")] = ev
        if not (ev[0] > 0 and ev[0] + ev[1] >= ev[2] * (1 - 1e-9)):
            problems.append("%s: inertia not physically valid, eig=%s" % (l.get("name"), ev))
    for n, ev in principal.items():
        if "left" not in n:
            continue
        mirror = n.replace("left", "right")
        if mirror in principal:
            rel = max(abs(a - b) / max(a, b) for a, b in zip(ev, principal[mirror])) * 100
            print("   L/R principal inertia mismatch %-28s %5.2f%%%s"
                  % (n.replace("robot_left_leg_", ""), rel, "  <-- CHECK" if rel >= 3 else ""))
            if rel >= 3:
                problems.append("%s vs %s principal inertia differ by %.1f%%" % (n, mirror, rel))
    return links, joints, problems


def render(urdf_path, png_path):
    import mujoco
    from PIL import Image

    # MuJoCo's URDF loader drops visual geoms unless told otherwise; use a
    # render-only copy with that compiler flag so both visual and collision show.
    tmp = urdf_path.replace(".urdf", "_render_tmp.urdf")
    txt = open(urdf_path, encoding="utf-8").read()
    end_of_robot_tag = txt.index(">", txt.index("<robot")) + 1
    txt = (txt[:end_of_robot_tag]
           + '<mujoco><compiler discardvisual="false"/></mujoco>'
           + txt[end_of_robot_tag:])
    with open(tmp, "w", encoding="utf-8") as f:
        f.write(txt)
    m = mujoco.MjModel.from_xml_path(tmp)
    os.remove(tmp)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    # group 0 = collision (has contype), group 1 = visual (no contact)
    for i in range(m.ngeom):
        m.geom_group[i] = 0 if m.geom_contype[i] else 1
    r = mujoco.Renderer(m, height=460, width=560)
    rows = []
    for groups in ((1,), (0,), (0, 1)):
        opt = mujoco.MjvOption()
        opt.geomgroup[:] = 0
        for g in groups:
            opt.geomgroup[g] = 1
        imgs = []
        for az, el in ((135, -15), (90, -10), (180, -10)):
            cam = mujoco.MjvCamera()
            cam.lookat[:] = [0, 0, -0.1]
            cam.distance = 1.1
            cam.azimuth = az
            cam.elevation = el
            r.update_scene(d, camera=cam, scene_option=opt)
            imgs.append(r.render().copy())
        rows.append(np.concatenate(imgs, axis=1))
    Image.fromarray(np.concatenate(rows, axis=0)).save(png_path)
    return m


def parse_args():
    import argparse

    p = argparse.ArgumentParser(
        description="Post-process an ACDC4Robot (Fusion 360) URDF export for MuJoCo / Isaac / viewers.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("urdf", help="raw ACDC4Robot export (<name>.urdf, meshes/ next to it)")
    p.add_argument("--collision", choices=["coacd", "primitive", "keep"], default="coacd",
                   help="coacd: convex decomposition pieces (default); primitive: one fitted "
                        "box/cylinder/sphere per link; keep: leave the exported mesh collision")
    p.add_argument("--rotate-links", default="",
                   help="comma-separated links whose visual/collision/inertial are rotated in "
                        "place (fixes a Fusion component frame that is rotated vs. the robot)")
    p.add_argument("--roll", type=float, default=0.0, help="roll angle in degrees for --rotate-links")
    p.add_argument("--pitch", type=float, default=0.0, help="pitch angle in degrees for --rotate-links")
    p.add_argument("--yaw", type=float, default=0.0, help="yaw angle in degrees for --rotate-links")
    p.add_argument("--root-yaw", type=float, default=0.0,
                   help="rotate the whole robot about Z by this many degrees (root link "
                        "contents + root joints) to get +X forward / +Y left")
    p.add_argument("--coacd-threshold", type=float, default=COACD_THRESHOLD,
                   help="CoACD concavity threshold, lower = tighter fit, more pieces")
    p.add_argument("--coacd-max-hulls", type=int, default=COACD_MAX_HULLS, help="max pieces per link")
    p.add_argument("--coacd-max-verts", type=int, default=COACD_MAX_HULL_VERTS,
                   help="max vertices per piece (PhysX convex meshes need <= 64 for cooking)")
    p.add_argument("--no-render", action="store_true", help="skip the MuJoCo load + PNG render")
    return p.parse_args()


def main():
    a = parse_args()
    src = os.path.abspath(a.urdf)
    folder = os.path.dirname(src)
    stem = os.path.splitext(os.path.basename(src))[0]
    out = os.path.join(folder, stem + "_fixed.urdf")
    png = os.path.join(folder, stem + "_fixed.png")

    tree = ET.parse(src)
    root = tree.getroot()

    removed = dedupe_joints(root)
    print("1. duplicate joints removed:", removed)

    print("2. mesh decimation:")
    decimate_meshes(os.path.join(folder, "meshes"))

    fixed_root = fix_root_inertial(root)
    print("3a. root link inertial re-expressed in visual frame:", fixed_root)

    rotate = [n for n in a.rotate_links.split(",") if n]
    if rotate:
        Rfix = rpy_to_R(math.radians(a.roll), math.radians(a.pitch), math.radians(a.yaw))
        rotated = []
        for link in root.findall("link"):
            if link.get("name") in rotate:
                rotate_link_contents(link, Rfix)
                rotated.append(link.get("name"))
        print("3. rotated rpy (%g, %g, %g) deg:" % (a.roll, a.pitch, a.yaw), rotated)
        missing = set(rotate) - set(rotated)
        if missing:
            print("   WARNING: --rotate-links not found in URDF:", sorted(missing))

    if a.root_yaw:
        res = rotate_root_frame(root, rpy_to_R(0, 0, math.radians(a.root_yaw)))
        print("3c. root frame yawed %g deg:" % a.root_yaw,
              "%s + %d root joints" % res if res else "SKIPPED (no single root)")

    if a.collision == "coacd":
        print("3b. collision -> convex decomposition (CoACD) per link:")
        convex_decomposition_collisions(root, os.path.join(folder, "meshes"),
                                        a.coacd_threshold, a.coacd_max_hulls, a.coacd_max_verts)
    elif a.collision == "primitive":
        print("3b. collision -> fitted primitive per link (mm):")
        for name, desc in primitive_collisions(root, os.path.join(folder, "meshes")):
            print("    %-32s %s" % (name, desc))

    links, joints, problems = validate(root)
    print("4. links: %d  joints: %d" % (len(links), len(joints)))
    for p in problems:
        print("   PROBLEM:", p)
    if problems:
        sys.exit(1)

    ET.indent(tree, space="    ")
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print("\nwrote:", out)

    if not a.no_render:
        m = render(out, png)
        print("5. MuJoCo load OK (bodies=%d, joints=%d)" % (m.nbody, m.njnt))
        print("render:", png)


if __name__ == "__main__":
    main()

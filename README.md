# acdc4robot-fix

Fixes and a post-processing script for **Fusion 360 → URDF export with
[ACDC4Robot](https://github.com/bionicdl-sustech/ACDC4Robot)**, so the URDF
actually loads in MuJoCo / Isaac Sim / web viewers and is usable for sim2real.

Built while exporting a 12-link bipedal robot. Every fix below was found by
symptom, traced to a root cause, and verified numerically — the symptom /
cause / fix tables are written so you (or an AI assistant) can match your
own error message and apply the fix directly.

```
Fusion 360 ──ACDC4Robot (+patch)──▶ robot.urdf + meshes/
                                          │
                       fix_acdc_export.py │  dedupe joints · decimate meshes
                                          │  fix root inertial · rotate frames
                                          │  convex-decomposition collision
                                          │  validate tree + inertia · render
                                          ▼
                                   robot_fixed.urdf  ──▶ MuJoCo / Isaac Lab
```

## Quick start

```bash
pip install numpy scipy trimesh fast-simplification coacd mujoco pillow

# 1. (once) patch the add-in — see patches/README.md
# 2. export from Fusion 360 with ACDC4Robot (URDF)
# 3. post-process
python3 fix_acdc_export.py my_robot/my_robot.urdf
#    -> my_robot/my_robot_fixed.urdf, my_robot_fixed.png (visual / collision / overlay)
```

Options: `--collision coacd|primitive|keep`, `--rotate-links L1,L2 --roll 90`,
`--coacd-max-hulls 8`, `--no-render`. See `python3 fix_acdc_export.py --help`.

The script never modifies the input URDF; it writes `<name>_fixed.urdf` next
to it, and refuses to write if validation fails.

---

## Problems this fixes (symptom → cause → fix)

### 1. Viewer shows nothing / `check_urdf` fails / joints reference links that don't exist

**Symptom.** URDF + meshes load into a viewer and nothing renders. Inspecting
the file: far more `<link>` elements than joints, dozens of root links, and the
links named in `<parent>`/`<child>` have **no `<link>` element at all**.

```
links: 147 | joints: 11
joint-referenced links missing a <link> element: base_link, left_thigh, ...
```

**Cause.** ACDC4Robot's `get_link_joint_list()` treats only *leaf* occurrences
(no child components) as links. If your link components contain
sub-components (motors, bearings, brackets — the normal way to build a CAD
assembly), every jointed link is skipped and every sub-part becomes an orphan
link. The code comment says "nested components problem ... not fully tested".

**Fix.** `patches/acdc4robot_nested_components.patch` — a link is any
occurrence that participates in a joint; its whole subtree is exported as one
rigid body (Fusion's STL export and `getPhysicalProperties` of an occurrence
already include children). Verified: link count, mass sum, left/right
symmetry, MuJoCo load. Also submitted upstream as a PR.

**Do not** try to flatten sub-components with the Fusion API
(`BRepBody.moveToComponent` → `InternalValidationError: cut_raw()`, and
`copyToComponent` + delete destroys every joint anchored on sub-component
geometry). We lost all joints that way.

### 2. Every joint appears twice → link has two parents

**Symptom.** 22 joints for an 11-joint robot, identical pairs.
**Cause.** `root.allJoints` / `allAsBuiltJoints` returned the same joint twice
(seen once, after copying the design). **Fix.** dedupe by name — in the patch
and in the script (`dedupe_joints`).

### 3. MuJoCo: `stl_decoder: number of faces should be between 1 and 200000`

**Cause.** A high-detail vendor motor CAD inside one link (254k faces).
**Fix.** script step 2 decimates any STL over 200k faces to 120k
(`trimesh` + `fast-simplification`), in place.

### 4. Root link's inertia box is rotated relative to its mesh (children are fine)

**Symptom.** Inertia visualisation fits every leg link but the root link
(`base_link`) is off by ~90°. Comparing a uniform-density CoM computed from the
mesh with the URDF: **only the root link has a sign flip**.

**Cause (ACDC4Robot bug).** For the root link, `urdf.py` writes the *visual*
origin as `link.pose` (link frame = Fusion world), but `link.py
get_CoM_sdf()` computes the *inertial* CoM/tensor in the component's own
frame (`L_R_w · (CoM_w − Lo_w)`). Two different frames for the same link.
Child links go through the parent-joint path and are consistent.

**Fix.** script step 3a (`fix_root_inertial`): re-express the root inertial in
the visual frame — `CoM' = R·CoM + t`, `I' = R·I·Rᵀ` with `(R, t)` = root
visual origin. Reported upstream as an issue.

### 5. A link is rotated 90° in every viewer (and it's not a Y-up/Z-up issue)

**Symptom.** One link (often `base_link`) sits rotated relative to the rest;
MuJoCo and a web viewer agree, so it is the data, not the viewer.
**Cause.** The Fusion component's own coordinate frame is rotated relative to
the robot; ACDC4Robot trusts component frames.
**Fix.** `--rotate-links base_link,torso --roll 90`: rotates visual /
collision / inertial *contents* of those links in place; joints untouched.
The proper fix is re-aligning the component origin in Fusion.

### 6. Inertia box misaligned only in a web viewer, fine in MuJoCo

**Cause.** Some viewers (e.g. viewer.robotsfan.com) ignore the inertial
origin's `rpy`. **Fix.** the script never writes a rotated inertial frame; any
rotation is folded into the tensor (`I' = R·I·Rᵀ`) and `rpy` stays `0 0 0`.

### 7. Root link mass shows as base + child in the viewer

Not a bug: viewers / MuJoCo / Isaac merge links joined by a `fixed` joint into
one rigid body. Check the URDF text for the per-link mass.

### 8. Collision geometry is the full visual mesh (bad for self-collision and speed)

ACDC4Robot copies the visual mesh into `<collision>`. Simulators convexify
mesh collisions, so concave parts get filled and links overlap at the zero
pose; self-collision becomes unusable.

**Fix.** `--collision coacd` (default): [CoACD](https://github.com/SarahWeiii/CoACD)
convex decomposition, ≤12 pieces per link, ≤64 vertices per piece (PhysX's
convex cooking limit — note CoACD only honours `max_ch_vertex` when
`decimate=True`). `--collision primitive` fits one box / cylinder / sphere per
link using the classification rules from
[fusion2URDF](https://github.com/Adriaeik/fusion2URDF) (dimension pattern →
volume fill → revolute-axis hint), but only picks a cylinder when the
cross-section is actually round, so feet stay flat boxes.

### 9. Cylinder collision lying sideways (pitch = ±90°)

**Cause.** Naive rotation-matrix → rpy conversion (`asin`/`atan2`) is wrong at
gimbal lock. **Fix.** `scipy.spatial.transform.Rotation.as_euler("xyz")`.

---

## What the validation catches

Run on every export; the script exits non-zero and writes nothing on failure.

| check | catches |
|---|---|
| every joint parent/child has a `<link>`; exactly one root; no link with two parents | #1, #2 |
| inertia positive definite and I₁ + I₂ ≥ I₃ | broken tensors |
| **left/right principal moments agree within 3 %** | a mis-placed member that leaves mass and CoM correct but inertia 100× off (found this way in the fusion2URDF attempt) |
| MuJoCo load + 3-view render, rows = visual / collision / overlay | frame and collision mistakes you can see |

Inertia never shows in a render. The symmetry check is the one that is
independent of the exporter's own formula, so it cannot be fooled by
comparing an exporter against itself.

---

## Repository layout

```
fix_acdc_export.py                       post-processing script (numpy/scipy/trimesh/coacd/mujoco)
patches/acdc4robot_nested_components.patch  ACDC4Robot add-in fix (git apply)
patches/README.md                        how to apply
docs/cad2urdf.ko.md                      full write-up in Korean, incl. the earlier fusion2URDF attempt
docs/example_render.png                  what the render output looks like
llms.txt                                 condensed version for AI assistants
```

## Upstream status

- Nested-components fix → PR [ACDC4Robot/Fusion360#17](https://github.com/ACDC4Robot/Fusion360/pull/17)
- Root-link inertial frame bug → issue [ACDC4Robot/Fusion360#18](https://github.com/ACDC4Robot/Fusion360/issues/18)

Until merged, apply the patch locally (see `patches/README.md`) and run the script.

## Related

- ACDC4Robot (upstream, moved from `bionicdl-sustech/ACDC4Robot`): https://github.com/ACDC4Robot/Fusion360
- fusion2URDF (the exporter we used first; its `fit_primitive` rules and `check_inertia.py` are ported here): https://github.com/Adriaeik/fusion2URDF
- CoACD: https://github.com/SarahWeiii/CoACD
- URDF+ (closed loops): https://arxiv.org/abs/2411.19753

## License

MIT

"""Create the small public nested-assembly fixture for ACDC4Robot.

The generated design has three jointed links, a +90 degree rotated base link,
two occurrences of the same arm component, a nested hidden payload, and two
rigid as-built joints.  It is exported locally as ``acdc_nested_fixture.f3d``.
"""

from __future__ import annotations

import math
import os
import traceback

import adsk.core
import adsk.fusion


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "acdc_nested_fixture.f3d")
ERROR_PATH = os.path.join(SCRIPT_DIR, "fixture_create_error.txt")


def _transform(rotation_x=0.0, translation=(0.0, 0.0, 0.0)):
    matrix = adsk.core.Matrix3D.create()
    if rotation_x:
        matrix.setToRotation(
            rotation_x,
            adsk.core.Vector3D.create(1.0, 0.0, 0.0),
            adsk.core.Point3D.create(0.0, 0.0, 0.0),
        )
    matrix.translation = adsk.core.Vector3D.create(*translation)
    return matrix


def _add_box(component, name, center, size):
    manager = adsk.fusion.TemporaryBRepManager.get()
    bounds = adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(*center),
        adsk.core.Vector3D.create(1.0, 0.0, 0.0),
        adsk.core.Vector3D.create(0.0, 1.0, 0.0),
        size[0],
        size[1],
        size[2],
    )
    transient = manager.createBox(bounds)
    base_feature = component.features.baseFeatures.add()
    if not base_feature.startEdit():
        raise RuntimeError("Could not enter BaseFeature edit mode")
    try:
        body = component.bRepBodies.add(transient, base_feature)
        if body is None:
            raise RuntimeError("Could not persist fixture box body")
        body.name = name
    finally:
        base_feature.finishEdit()
    return body


def _add_rigid_joint(root, child, parent, name):
    joint_input = root.asBuiltJoints.createInput(child, parent, None)
    joint_input.setAsRigidJointMotion()
    joint = root.asBuiltJoints.add(joint_input)
    joint.name = name
    return joint


def build_fixture(app):
    document = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    # Fusion forbids renaming a document's root component.  The exported
    # archive filename identifies the fixture; link component names below are
    # stable and are also used to find the open fixture document.  Keep the
    # default parametric mode because AsBuiltJointInput requires it.

    base = root.occurrences.addNewComponent(
        _transform(rotation_x=math.pi / 2.0)
    )
    base.component.name = "base_link"
    _add_box(base.component, "base_asymmetric_box", (2.0, 3.0, 4.0), (8.0, 4.0, 2.0))

    hidden = base.component.occurrences.addNewComponent(
        _transform(translation=(1.0, -2.0, 0.5))
    )
    hidden.component.name = "hidden_payload"
    _add_box(hidden.component, "hidden_payload_box", (0.0, 0.0, 0.0), (2.0, 3.0, 1.0))
    hidden.isLightBulbOn = False

    arm_one = root.occurrences.addNewComponent(
        _transform()
    )
    arm_one.component.name = "arm_link"
    _add_box(arm_one.component, "arm_box", (0.0, 0.0, 0.0), (5.0, 2.0, 3.0))

    arm_two = root.occurrences.addExistingComponent(
        arm_one.component,
        _transform(),
    )

    _add_rigid_joint(root, arm_one, base, "base_to_arm_one")
    _add_rigid_joint(root, arm_two, base, "base_to_arm_two")

    output = os.path.abspath(OUTPUT_PATH)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    options = design.exportManager.createFusionArchiveExportOptions(output, root)
    if not design.exportManager.execute(options):
        raise RuntimeError(f"Fusion archive export failed: {output}")

    app.activeViewport.fit()
    return document, output


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        _, output = build_fixture(app)
        ui.messageBox(
            "Public ACDC4Robot fixture created.\n\n"
            "Links: 3\n"
            "Joints: 2 rigid as-built joints\n"
            "Nested hidden payload: yes\n"
            "Repeated arm component: yes\n"
            "base_link rotation: +90 deg about world X\n"
            f"Archive: {output}"
        )
    except Exception:
        details = traceback.format_exc()
        os.makedirs(os.path.dirname(ERROR_PATH), exist_ok=True)
        with open(ERROR_PATH, "w", encoding="utf-8") as stream:
            stream.write(details)
        ui.messageBox(
            "create_fixture.py failed; full traceback written to:\n\n"
            + ERROR_PATH
        )


def stop(context):
    pass

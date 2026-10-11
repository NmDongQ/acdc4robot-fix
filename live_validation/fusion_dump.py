"""Dump live Fusion assembly truth for ACDC4Robot validation.

Run this as a Fusion Python script with the robot/fixture design active.  It
writes ``~/fusion_props.json`` using the versioned ``fusion_props/v1`` schema.

All exported values use SI units.  Inertia is shifted from Fusion's world
origin to the center of mass with the parallel-axis theorem, while retaining
world-aligned axes.
"""

from __future__ import annotations

from collections import Counter
from datetime import datetime, timezone
import json
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


OUTPUT_PATH = os.path.expanduser("~/fusion_props.json")
ACCURACY = adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy
JOINT_TYPE_NAMES = {
    0: "rigid",
    1: "revolute",
    2: "slider",
    3: "cylindrical",
    4: "pin-slot",
    5: "planar",
    6: "ball",
}


def _items(collection):
    try:
        return list(collection)
    except TypeError:
        return [collection.item(index) for index in range(collection.count)]


def _valid_filename(value):
    """Mirror ACDC4Robot.core.utils.get_valid_filename at main@f1152d7."""
    value = str(value).strip()
    value = value.replace("+", "__").replace(":", "_").replace(" ", "-")
    return re.sub(r"(?u)[^-\w.]", "", value)


def _urdf_link_name(occurrence):
    if occurrence.component.name == "base_link":
        return "base_link"
    return _valid_filename(occurrence.fullPathName)


def _same_api_object(left, right):
    if left is right:
        return True
    try:
        return bool(left == right)
    except Exception:
        return False


def _unique_joints(root):
    raw = _items(root.allJoints) + _items(root.allAsBuiltJoints)
    unique = []
    for joint in raw:
        if not any(_same_api_object(joint, previous) for previous in unique):
            unique.append(joint)
    return raw, unique


def _discover_links(root, joints):
    """Mirror the validated occurrence discovery merged in PR #17."""
    occurrences = _items(root.allOccurrences)
    endpoints = {}
    errors = []
    for joint in joints:
        for occurrence in (joint.occurrenceOne, joint.occurrenceTwo):
            if occurrence is None:
                errors.append(
                    f"Joint {joint.name} connects to root/ground; use a base_link occurrence."
                )
                continue
            if not occurrence.isLightBulbOn:
                errors.append(
                    f"Joint {joint.name} references hidden occurrence "
                    f"{occurrence.fullPathName}."
                )
            endpoints.setdefault(occurrence.fullPathName, occurrence)

    if not joints:
        return [
            occurrence
            for occurrence in occurrences
            if occurrence.isLightBulbOn
            and occurrence.childOccurrences.count == 0
            and occurrence.component.bRepBodies.count
        ]

    roots = list(endpoints)
    for path in roots:
        for other in roots:
            if path != other and path.startswith(other + "+"):
                errors.append(
                    f"Overlapping link subtrees {other} and {path}; move joint "
                    "endpoints to non-overlapping link components."
                )
    for occurrence in occurrences:
        if not occurrence.isLightBulbOn or not occurrence.component.bRepBodies.count:
            continue
        path = occurrence.fullPathName
        owners = [
            owner for owner in roots if path == owner or path.startswith(owner + "+")
        ]
        if len(owners) != 1:
            errors.append(
                f"Visible geometry {path} must belong to exactly one jointed link "
                f"subtree; found {len(owners)}."
            )
    if errors:
        raise ValueError("\n".join(dict.fromkeys(errors)))
    return list(endpoints.values())


def _point_m(point):
    return [point.x * 0.01, point.y * 0.01, point.z * 0.01]


def _matrix_m(matrix):
    rows = []
    for row in range(4):
        values = []
        for column in range(4):
            value = matrix.getCell(row, column)
            if column == 3 and row < 3:
                value *= 0.01
            values.append(value)
        rows.append(values)
    return {"storage": "row-major", "matrix4x4": rows}


def _physical_record(properties, include_inertia=True):
    center = _point_m(properties.centerOfMass)
    record = {
        "mass_kg": properties.mass,
        "volume_m3": properties.volume * 1.0e-6,
        "center_of_mass_world_m": center,
    }
    if not include_inertia:
        return record

    ok, w_xx, w_yy, w_zz, w_xy, w_yz, w_xz = (
        properties.getXYZMomentsOfInertia()
    )
    if not ok:
        raise RuntimeError("Fusion getXYZMomentsOfInertia() returned false")
    x, y, z = center
    mass = properties.mass
    record["inertia_com_world_kg_m2"] = {
        "xx": w_xx * 1.0e-4 - mass * (y * y + z * z),
        "yy": w_yy * 1.0e-4 - mass * (x * x + z * z),
        "zz": w_zz * 1.0e-4 - mass * (x * x + y * y),
        "xy": w_xy * 1.0e-4 + mass * x * y,
        "yz": w_yz * 1.0e-4 + mass * y * z,
        "xz": w_xz * 1.0e-4 + mass * x * z,
    }
    return record


def _body_properties(design, body):
    get_properties = getattr(body, "getPhysicalProperties", None)
    if callable(get_properties):
        return get_properties(ACCURACY)

    inputs = adsk.core.ObjectCollection.create()
    inputs.add(body)
    return design.physicalProperties(inputs, ACCURACY)


def _effective_body_visibility(occurrence, body):
    try:
        return bool(body.isVisible)
    except (AttributeError, RuntimeError):
        try:
            occurrence_visible = bool(occurrence.isVisible)
        except (AttributeError, RuntimeError):
            occurrence_visible = bool(occurrence.isLightBulbOn)
        return bool(body.isLightBulbOn) and occurrence_visible


def _body_records(design, link, all_occurrences, warnings):
    owner_path = link.fullPathName
    subtree = [
        occurrence
        for occurrence in all_occurrences
        if occurrence.fullPathName == owner_path
        or occurrence.fullPathName.startswith(owner_path + "+")
    ]
    records = []
    for occurrence in subtree:
        for body in _items(occurrence.bRepBodies):
            try:
                props = _body_properties(design, body)
                record = _physical_record(props, include_inertia=False)
            except Exception as exc:
                warnings.append(
                    f"Body physical properties unavailable for "
                    f"{occurrence.fullPathName}/{body.name}: {exc}"
                )
                continue
            try:
                occurrence_visible = bool(occurrence.isVisible)
            except (AttributeError, RuntimeError):
                occurrence_visible = bool(occurrence.isLightBulbOn)
            record.update(
                {
                    "name": body.name,
                    "occurrence_path": occurrence.fullPathName,
                    "component_name": occurrence.component.name,
                    "body_visible": bool(body.isLightBulbOn),
                    "occurrence_visible": occurrence_visible,
                    "effective_visible": _effective_body_visibility(occurrence, body),
                }
            )
            records.append(record)
    records.sort(key=lambda item: (item["occurrence_path"], item["name"]))
    return records


def _joint_frame(joint):
    """Return a world-space joint frame, including current API fallbacks."""
    try:
        if hasattr(joint, "geometry") and joint.geometry is not None:
            geometry = joint.geometry
            origin = geometry.origin
            frame = adsk.core.Matrix3D.create()
            frame.setWithCoordinateSystem(
                origin,
                geometry.secondaryAxisVector,
                geometry.thirdAxisVector,
                geometry.primaryAxisVector,
            )
            return frame
    except (AttributeError, RuntimeError):
        pass

    for attribute in ("geometryTwoTransform", "geometryOneTransform", "transform"):
        try:
            frame = getattr(joint, attribute)
            if frame is not None:
                return frame
        except (AttributeError, RuntimeError):
            continue

    try:
        geometry = joint.geometryOrOriginTwo
        if isinstance(geometry, adsk.fusion.JointOrigin):
            geometry = geometry.geometry
        if geometry is not None:
            frame = adsk.core.Matrix3D.create()
            frame.setWithCoordinateSystem(
                geometry.origin,
                geometry.secondaryAxisVector,
                geometry.thirdAxisVector,
                geometry.primaryAxisVector,
            )
            return frame
    except (AttributeError, RuntimeError):
        pass
    return None


def _joint_axis(joint):
    try:
        joint_type = joint.jointMotion.jointType
        if joint_type in (1, 3):
            vector = joint.jointMotion.rotationAxisVector
        elif joint_type == 2:
            vector = joint.jointMotion.slideDirectionVector
        else:
            return None
        return [vector.x, vector.y, vector.z]
    except (AttributeError, RuntimeError):
        return None


def build_dump(app, design):
    root = design.rootComponent
    all_occurrences = _items(root.allOccurrences)
    raw_joints, joints = _unique_joints(root)
    links = _discover_links(root, joints)
    warnings = []

    name_counts = Counter(joint.name for joint in joints)
    root_props = root.getPhysicalProperties(ACCURACY)
    root_record = _physical_record(root_props, include_inertia=False)
    root_record["name"] = root.name

    link_records = {}
    for occurrence in links:
        properties = occurrence.getPhysicalProperties(ACCURACY)
        record = _physical_record(properties)
        record.update(
            {
                "full_path_name": occurrence.fullPathName,
                "component_name": occurrence.component.name,
                "urdf_name": _urdf_link_name(occurrence),
                "transform_local_to_world": _matrix_m(occurrence.transform2),
                "visible": bool(occurrence.isLightBulbOn),
                "bodies": _body_records(
                    design, occurrence, all_occurrences, warnings
                ),
            }
        )
        link_records[occurrence.fullPathName] = record

    joint_records = []
    for joint in joints:
        try:
            joint_type_number = joint.jointMotion.jointType
        except (AttributeError, RuntimeError):
            joint_type_number = None
        frame = _joint_frame(joint)
        if frame is None:
            origin = None
            warnings.append(f"Joint origin unavailable for {joint.name}")
        else:
            origin = [
                frame.getCell(0, 3) * 0.01,
                frame.getCell(1, 3) * 0.01,
                frame.getCell(2, 3) * 0.01,
            ]
        joint_records.append(
            {
                "name": joint.name,
                "type": JOINT_TYPE_NAMES.get(
                    joint_type_number, f"unknown-{joint_type_number}"
                ),
                "occurrence_one_path": (
                    joint.occurrenceOne.fullPathName
                    if joint.occurrenceOne is not None
                    else None
                ),
                "occurrence_two_path": (
                    joint.occurrenceTwo.fullPathName
                    if joint.occurrenceTwo is not None
                    else None
                ),
                "origin_world_m": origin,
                "axis_world_unit": _joint_axis(joint),
            }
        )
    joint_records.sort(key=lambda item: item["name"])

    return {
        "schema": "fusion_props/v1",
        "units": {
            "length": "m",
            "mass": "kg",
            "volume": "m^3",
            "inertia": "kg*m^2",
        },
        "accuracy": "VeryHighCalculationAccuracy",
        "document": {
            "name": app.activeDocument.name,
            "product_type": str(app.activeProduct.productType),
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        },
        "summary": {
            "link_count": len(links),
            "joint_records_seen": len(raw_joints),
            "joint_count_unique": len(joints),
            "duplicate_joint_record_count": len(raw_joints) - len(joints),
            "duplicate_joint_names": sorted(
                name for name, count in name_counts.items() if count > 1
            ),
        },
        "root_component": root_record,
        "links": link_records,
        "joints": joint_records,
        "warnings": warnings,
    }


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        if design is None:
            raise RuntimeError("Open a Fusion design before running fusion_dump.py")
        result = build_dump(app, design)
        with open(OUTPUT_PATH, "w", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        summary = result["summary"]
        ui.messageBox(
            "fusion_props/v1 written successfully.\n\n"
            f"Links: {summary['link_count']}\n"
            f"Joint records: {summary['joint_records_seen']}\n"
            f"Unique joints: {summary['joint_count_unique']}\n"
            f"Duplicate records: {summary['duplicate_joint_record_count']}\n"
            f"Output: {OUTPUT_PATH}"
        )
    except Exception:
        ui.messageBox("fusion_dump.py failed:\n\n" + traceback.format_exc())


def stop(context):
    pass

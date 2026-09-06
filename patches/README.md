# ACDC4Robot add-in patch

`acdc4robot_nested_components.patch` changes one function,
`get_link_joint_list()` in `Add-IN/ACDC4Robot/commands/ACDC4Robot/acdc4robot.py`:

- a **link** is any occurrence that participates in a joint (`occurrenceOne` /
  `occurrenceTwo` of every `Joint` and `AsBuiltJoint`), exported with its whole
  subtree — nested sub-components no longer need flattening;
- joints returned twice by `allJoints` / `allAsBuiltJoints` are deduplicated
  by name;
- designs with no joints fall back to the old "leaf occurrence with bodies" rule.

Made against upstream commit `1acbfc1` (2026-08-23).

## Apply

Against a checkout of the add-in repository:

```bash
git clone https://github.com/ACDC4Robot/Fusion360.git
cd Fusion360
git apply /path/to/acdc4robot_nested_components.patch
# then copy Add-IN/ACDC4Robot into Fusion's add-in folder (see below)
```

Against an already installed add-in (macOS path shown; Windows:
`%appdata%\Autodesk\Autodesk Fusion 360\API\AddIns`):

```bash
cd "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/ACDC4Robot"
patch -p3 < /path/to/acdc4robot_nested_components.patch
```

Restart Fusion 360 afterwards — the add-in's Python modules are cached, so
Stop/Run in the Add-Ins dialog is not enough.

Updating the add-in from the Autodesk App Store overwrites the file; re-apply.

## Modelling rule this relies on

Create your joints **between the link components** (thigh ↔ calf), not
between sub-parts inside them. The occurrences named by the joint are what
become links.

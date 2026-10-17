# ROSOrin Pro vendor materials

This directory separates the official ROSOrin Pro simulation download from code,
configuration, maps, experiment logs, and documentation created for this task.

## Local layout

- `original/`: unchanged downloads from the vendor Feature Package.
- `upstream/src/simulations/`: extracted source used as the upstream simulation
  dependency. It contains `robot_gazebo` and `rosorin_description`.

Both directories are intentionally ignored by Git. They are vendor-provided files
with no repository license confirmed here. A fresh clone should download the
Feature Package again and verify the hashes below before extracting it.

## Provenance

- Documentation: <https://wiki.hiwonder.com/projects/rosorin-pro/en/latest/docs/9_Gazebo_Simulation.html>
- Feature Package folder: <https://drive.google.com/drive/folders/1sq3698U-EX7a4gbjmGnwYbptlJ3B8aMM?usp=sharing>
- Retrieved: 2026-10-12

| File | Size | SHA-256 |
| --- | ---: | --- |
| `original/simulations.zip` | 7,629,613 bytes | `a4d23a901dd24472cfdb4655304fd4409bdb5982a7ebdea13f70ec42aef8dad2` |
| `original/.typerc` | 1,228 bytes | `0c1f9c4b256beed7a1e1ffefcb3149577fcf6730284f1cbc2bbf2c09adee6320` |

The archive integrity test passed after download. Do not automatically append the
vendor `.typerc` to `~/.bashrc`: it sets ROS and Cyclone DDS environment values
that should first be checked against the local machine.

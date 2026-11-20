# Changelog

Each version's notes show in the app under **What's new**, and on its GitHub release.

## 1.0.3

- New: the Windows installer shows who made it (Aaryan Roy, FRC Team 1089) and asks you to accept the terms of use before installing. The terms are also included in the Mac, Linux and portable downloads as TERMS.txt.

## 1.0.2

- Fixed: saving could fail with a "FileSystemException" error, mostly while dragging a path or editing a backup path. Saves now wait their turn and retry.
- Fixed: in Pivot view, the robot now follows the speed limits (constraint zones), rotation targets, point-towards zones and event markers from both the main path and the pivot path. Before, they were dropped from the pivot route.
- Fixed: Pivot view now shows the pivot path's end (red robot outline and end point), plus the waypoints and rotations of the main path up to where the pivot breaks off.
- New: one-click update on Windows. When a new version is out, click **Update** and the app installs it and reopens.
- New: **What's new** shows after an update. You can also open it from the menu, next to the version number.

## 1.0.1

- Released by mistake with the same code as 1.0.0. Use 1.0.2.

## 1.0.0

- First release of PathPlanner Backup Paths.
- Robot in the way regions: wait, divert onto a backup path, or continue.
- Game piece regions: drive to a seen game piece with the intake running, then rejoin the path or divert.
- Dashboard toggles for divert, wait and pickup.
- PathplannerLib-BackupPaths vendordep and an example robot project.

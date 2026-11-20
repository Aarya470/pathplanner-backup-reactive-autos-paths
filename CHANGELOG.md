# Changelog

Each version's notes show in the app under **What's new**, and on its GitHub release.

## 1.0.1

- Fixed: saving could fail with a "FileSystemException" error, mostly while dragging a path or editing a backup path. Saves now wait their turn and retry.
- New: one-click update on Windows. When a new version is out, click **Update** and the app installs it and reopens.
- New: **What's new** shows after an update. You can also open it from the menu, next to the version number.

## 1.0.0

- First release of PathPlanner Backup Paths.
- Robot in the way regions: wait, divert onto a backup path, or continue.
- Game piece regions: drive to a seen game piece with the intake running, then rejoin the path or divert.
- Dashboard toggles for divert, wait and pickup.
- PathplannerLib-BackupPaths vendordep and an example robot project.

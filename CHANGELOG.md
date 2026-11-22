# Changelog

Each version's notes show in the app under **What's new**, and on its GitHub release.

## 1.2.0

- New: robot detection is built into the robot library. `PhotonRobotDetector` reads a PhotonVision object detection camera and reports robots on the field, so team robot code no longer needs its own camera code.
- New: use it anywhere in robot code, not just in autos: `isRobotAhead(meters)`, `isRobotSeen()`, `getClosestRobot()`, with each robot's distance, direction and field position.
- New: `ObstacleSettings.configure(detector, pose, stop)` connects the detector to path regions in one call.
- New: other cameras can be used by implementing `RobotDetector`.
- Changed: the example robot project uses the new detector. The dashboard toggles and fake test robot were removed from it.
- Update the robot library to PathplannerLib-BackupPaths 1.2.0. Teams using `PhotonRobotDetector` also need the photonlib vendordep.

## 1.1.0

- New: pickup zones for game piece regions. Choose **Inside a zone** on a game piece region, then drag the 4 corners of the orange zone on the field. While the robot is in that part of the path, any game piece seen inside the zone is picked up, and pieces outside it are ignored. Regions without a zone still use the max distance.
- Fixed: after a pickup, the robot now rejoins the path facing the way the path faces there and at the path's speed, instead of turning to face 0°.
- Fixed: event markers now fire on paths with regions, including after the robot rejoins the path or resumes after waiting.
- Fixed: regions later on the path are still watched after a pickup and rejoin.
- Update both the app and the robot library (PathplannerLib-BackupPaths 1.1.0) to use pickup zones. An older library ignores the zone and uses the max distance.

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

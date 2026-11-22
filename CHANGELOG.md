# Changelog

Each version's notes show in the app under **What's new**, and on its GitHub release.

## 1.4.0

- New: one simple setup for robot code. `BackupPaths.setup(pose, stop)`, then `BackupPaths.watchForRobots(camera)` and `BackupPaths.pickUpGamePieces(camera, intakeCommand, hasPiece, intakeSubsystem)`.
- New: `Camera` works with any camera. `Camera.limelight("name")` reads a Limelight neural detector pipeline with no extra library, `Camera.photon("name")` reads PhotonVision, and `Camera.fromAngles(...)` or `Camera.fromFieldPositions(...)` work with anything else.
- New: say where a camera sits in plain numbers with `mountedAt(forward, left, height, tilt down, facing)`, and which model classes are red or blue robots with `redClasses(...)` and `blueClasses(...)`.
- New: read any camera in robot code with `seesSomethingWithin(meters)`, `getClosest()` and `getSightings()`, and put it on a dashboard to see its status.
- Changed: the example robot project and the READMEs use the new setup. Code written for 1.2 and 1.3 still works.
- Fixed: PhotonVision distances for targets near the sides of the image. Placing a box on the floor now follows the exact camera ray, so a robot 46° off center is no longer placed over 1 m short.
- Fixed: in the auto editor's Continue view, each wait is now shown in its own path. Pivot view stops where the robot pivots, since the next path starts back on the main route.
- Fixed: the path editor goes back to the normal view when its pivot region changes or is deleted, instead of getting stuck or crashing.
- Fixed: older paths get their pickup zone saved when opened, so the robot uses the same zone the app shows.
- Fixed: zone corners can't be dragged so the zone folds over itself, changing a region's range keeps the zone's shape and can be undone, and redo works for region settings.
- Fixed: the example robot project now uses library 1.4.0, and `camera_check.py` works again, with `mountedAt` numbers.
- Changed: with `redClasses`/`blueClasses` set, only those classes count as robots. The Driver Station warns if paths with regions are built before `BackupPaths.setup(...)`.
- Update the robot library to PathplannerLib-BackupPaths 1.4.0. PhotonLib is only needed for PhotonVision cameras.

## 1.3.0

- Fixed: the Path Regions list no longer closes when you add a region. The new region opens and is selected right away, so its part of the path shows on the field.
- Changed: regions are easier to see. Every region shows as a yellow band on the path, and the selected or hovered one is thick and outlined, with a dot at each end.
- Changed: game piece regions always use a pickup zone. The **Within a distance** option is gone. Choosing **Game piece** makes an orange zone around the region, and older paths get one when opened.
- Changed: the 4 orange corner dots of every pickup zone always show on the field and can be dragged without opening the region first. A corner can be stretched up to about 3/4 of the region's length from the path (at least 1 m, at most 4 m).
- New: the auto editor has **Normal / Pivot / Continue** views when its paths have regions. Pivot shows and simulates each path's pivot route, and Continue shows the robot waiting at regions with a block timeout.
- New: `PhotonGamePieceDetector` in the robot library reads game pieces from a PhotonVision object detection camera, for game piece regions.
- Changed: the example robot project wires up pickups with the new detector, tells red and blue robots apart, and has an updated README. PhotonLib on the robot must match the PhotonVision version on the coprocessor.
- Update both the app and the robot library to PathplannerLib-BackupPaths 1.3.0.

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

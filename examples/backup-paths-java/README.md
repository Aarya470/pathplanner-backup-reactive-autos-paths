# Example robot project: PathPlanner Backup Paths

A small swerve robot (it runs in the simulator) that shows how to connect PhotonVision object detection cameras to path regions:

- `Score.path` has a **Robot in the way** region. When the robot detection camera sees a robot blocking it, the robot waits or diverts.
- `Pickup.path` has a **Game piece** region with an orange pickup zone. When the game piece camera sees a piece inside the zone, the robot drives to it, runs the intake, and rejoins the path.

## Try it

1. Open this folder in WPILib VS Code (2026).
2. Run **WPILib: Simulate Robot Code**, or deploy it to a robot after setting your team number in `.wpilib/wpilib_preferences.json`.
3. Open the paths in **PathPlanner Backup Paths**. Click a region on the right to see its part of the path in yellow, and drag the 4 orange dots to reshape a pickup zone.
4. Run `Example Auto` from the dashboard.

The dashboard shows a **Detection** and a **Game Pieces** table with each camera's status (`OK`, `Not calibrated`, `Wrong class ID 3`, and so on). Use them while setting up.

## Put it in your own robot code

1. Add the vendor libraries: **PathplannerLib-BackupPaths** (in place of PathplannerLib, remove `PathplannerLib-2026.x.json` from `vendordeps`) and **PhotonLib**.
2. Copy the `Detection` and `GamePieceDetection` classes from `Constants.java` and set them for your robot.
3. Copy `configureDetection()` and `configurePickups()` from `RobotContainer.java`. Change `swerve` to your drivetrain, and put your intake's command, beam break and subsystem in `configurePickups()`.
4. Call both **before** `AutoBuilder.buildAutoChooser()` or any `new PathPlannerAuto(...)`.

You can leave out either one: without `configurePickups()`, game piece regions do nothing, and without `configureDetection()`, robot regions do nothing.

## Constants to set

**Detection** (robot camera) and **GamePieceDetection** (game piece camera):

- `cameraName`: the camera's name in PhotonVision, exactly as typed there. Use two different cameras: two detectors reading the same camera take each other's frames.
- `robotToCamera`: where the camera sits, from the robot's center on the floor. x is forward, y is left, z is up (meters). In the rotation, pitch is the tilt (positive = tilted down) and yaw is the direction it faces (`Math.PI` = facing backward).
- `redRobotClassIds`, `blueRobotClassIds`: the class numbers your model uses for red and blue robots, so regions set to **opponents** or **teammates** can tell them apart.
- `pieceClassIds`: the class number or numbers your model uses for game pieces.
- `intakeFacingDeg`: `0` if the intake is on the front, `180` if it is on the back.

Calibrate each camera in PhotonVision at the resolution its detection pipeline uses. The code reads the lens numbers from that calibration, and reports nothing until it exists.

`camera_check.py` checks where a pixel lands on the floor and solves for the camera's tilt. Run `python camera_check.py --help` to see how.

## Version match

The PhotonLib version in `vendordeps/photonlib.json` (now `v2026.3.4`) must match the PhotonVision version on your coprocessor. If they differ, the camera shows as connected but no results come through. Update one to match the other.

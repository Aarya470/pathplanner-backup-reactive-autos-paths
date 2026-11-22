# Example robot project: PathPlanner Backup Paths

A small swerve robot (it runs in the simulator) that shows how to connect object detection cameras to path regions:

- `Score.path` has a **Robot in the way** region. When the robot camera sees a robot blocking it, the robot waits up to 1 s for it to move, then goes on.
- `Pickup.path` has a **Game piece** region with an orange pickup zone. When the piece camera sees a piece inside the zone, the robot drives to it, runs the intake, and rejoins the path.

## Try it

1. Open this folder in WPILib VS Code (2026).
2. Run **WPILib: Simulate Robot Code**, or deploy it to a robot after setting your team number in `.wpilib/wpilib_preferences.json`.
3. Open the paths in **PathPlanner Backup Paths**. Click a region on the right to see its part of the path in yellow, and drag the 4 orange dots to reshape a pickup zone.
4. Run `Example Auto` from the dashboard.

The dashboard shows **Robot Camera** and **Piece Camera**, each with its status (`OK`, `Wrong class ID 3`, `Not mounted`, ...), how many things it sees, and the closest distance. Use them while setting up.

## The setup, all in RobotContainer.java

```java
private final Camera robotCamera = Camera.photon("DetectionCam")
    .mountedAt(-0.41, 0.0, 0.13, 0, 180)   // forward m, left m, height m, tilt down deg, facing deg
    .redClasses(1).blueClasses(2);

private final Camera pieceCamera = Camera.photon("IntakeCam")
    .mountedAt(-0.36, 0.0, 0.30, 20, 180)
    .onlyClasses(0);

private void configureBackupPaths() {
  BackupPaths.setup(swerve::getPose, () -> swerve.driveRobotRelative(new ChassisSpeeds()));
  BackupPaths.watchForRobots(robotCamera);
  BackupPaths.intakeIsOnBack();
  BackupPaths.pickUpGamePieces(
      pieceCamera,
      () -> Commands.print("Intake running").andThen(Commands.idle()), // your intake command
      null);                                                           // your beam break, or null
      // add your intake subsystem as the last argument
}
```

`configureBackupPaths()` runs before `AutoBuilder.buildAutoChooser()`. It has to run before any auto is built.

## Using a Limelight or another camera

Change only how the camera is made. Everything else stays the same.

```java
Camera.limelight("limelight-front")                          // Limelight, no extra library
Camera.photon("DetectionCam")                                // PhotonVision, needs PhotonLib
Camera.fromAngles(() -> myTargets())                         // any camera that gives tx/ty angles
Camera.fromFieldPositions(() -> myFieldPositions())          // anything that gives field positions
```

With a Limelight, run a neural detector pipeline. Without PhotonVision cameras, you can remove `vendordeps/photonlib.json`.

## Put it in your own robot code

1. Add the **PathplannerLib-BackupPaths** vendor library in place of PathplannerLib (remove any `PathplannerLib*.json` from `vendordeps`). For PhotonVision cameras, add **PhotonLib** too.
2. Copy the two camera fields and `configureBackupPaths()` from `RobotContainer.java`.
3. Measure your cameras for `mountedAt`, from the robot's center on the floor:
   - forward and left, in meters (negative = behind or to the right)
   - the lens height above the floor, in meters
   - how far the camera is tilted down, in degrees (0 = level)
   - which way it faces, in degrees: 0 forward, 90 left, 180 back, -90 right
4. Set the class numbers your model uses: `redClasses` and `blueClasses` for robots (so regions can tell teammates from opponents), `onlyClasses` for game pieces.
5. Put in your intake's command, beam break (or `null`) and subsystem, for example `BackupPaths.pickUpGamePieces(pieceCamera, intake::intakeCommand, intake::hasPiece, intake)`. The subsystem matters: without it, the intake's default command keeps running during a pickup. Remove `intakeIsOnBack()` if your intake is on the front.

Use a separate `Camera` object for robots and for game pieces, each with its own classes, even if both read the same physical camera. With one shared object, game pieces would count as robots in the way and robots could be "picked up".

## PhotonVision notes

- The PhotonLib version in `vendordeps/photonlib.json` (now `v2026.3.4`) must match the PhotonVision version on your coprocessor. If they differ, nothing comes through. Update one to match the other.
- Calibrate each camera in PhotonVision at the resolution its detection pipeline uses. Distances then use the calibration, which is the most accurate. Without it, the angles are used, like a Limelight.

`camera_check.py` works out a camera's tilt from one tape measurement and checks where a target lands, for Limelight angles or PhotonVision pixels, with the same math as the robot code. Run `python camera_check.py --help` to see how.

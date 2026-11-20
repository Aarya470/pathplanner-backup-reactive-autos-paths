# Example robot project: PathPlanner Backup Paths

A small swerve robot (it runs in the simulator) that shows how to connect a PhotonVision object detection camera to backup path regions.

## Try it in the simulator

1. Open this folder in WPILib VS Code (2026).
2. Run **WPILib: Simulate Robot Code**.
3. Open the paths in **PathPlanner Backup Paths**: `Score` has a **Robot in the way** region, and `Pickup` has a **Game piece** region.
4. In the dashboard, turn on the fake robot or fake game piece from `SimDetectionSource`, place it near the path, and run `Example Auto`.

## Put it in your own robot code

Copy these into your project:

| File | What it does |
| --- | --- |
| `sensors/DetectionCamera.java` | Reads robots and game pieces from PhotonVision and places them on the field |
| `sensors/DetectionCameraConfig.java` | Loads the camera settings from `detection-camera.json` |
| `sensors/DetectionSource.java` | The interface both sources share |
| `sensors/SimDetectionSource.java` | Fake detections for the simulator |
| `commands/BackupPathOptions.java` | Dashboard dropdowns for divert, wait and pickup |
| `src/main/deploy/detection-camera.json` | Camera mount, model classes and pickup settings |

Then copy `configureDetection()` from `RobotContainer.java`, change `swerve` and `intake` to your own drivetrain and intake, and call it **before** you build your autos.

You also need these vendor libraries: **PathplannerLib-BackupPaths** (in place of PathplannerLib) and **PhotonLib**.

## Set up detection-camera.json

- `cameraName`: the camera's name in PhotonVision.
- `heightInches`, `forwardInches`, `leftInches`, `yawDegrees`, `pitchDownDegrees`: where the camera is mounted, measured from the robot center. Negative forward means behind the center.
- `classes`: what each model class number means: `gamePiece`, `robotRed`, `robotBlue`, `robot` (no bumper color) or `ignore`.
- `minConfidence`: detections below this are ignored.
- `intakeBeamBreakPort`: DIO port of a beam break that shows a piece is held, or `-1` for none.
- `intakeFacingDegrees`: `0` if the intake is on the front, `180` if it's on the back.

Calibrate the camera in PhotonVision at the resolution the detection pipeline uses. The code reads the lens numbers from that calibration.

`camera_check.py` checks where a pixel lands on the floor and solves for the camera's tilt. Run `python camera_check.py --help` to see how.

Before deploying to a real robot, set your team number in `.wpilib/wpilib_preferences.json`.

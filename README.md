# PathPlanner Backup Paths

A fork of [PathPlanner](https://github.com/mjansen4857/pathplanner) for FRC that adds **backup paths**: regions on a path that watch for robots or game pieces and change what your robot does in autonomous, on the fly.

Made by Aaryan Roy on FRC Team 1089. This is not an official PathPlanner release and is not affiliated with the PathPlanner project.

## What it adds

- **Robot in the way regions.** Draw a region on a path. If a robot is detected there, your robot waits, diverts onto a backup path you draw from the region edge, or continues.
- **Game piece regions.** If a game piece is seen inside the region's orange **pickup zone** while the robot is in the region, the robot drives to it with the intake running, then rejoins the main path or follows a divert path. Drag the zone's 4 corners on the field to shape it.
- **Both alliances are detected.** Teammates and opponents can both trigger a region.
- **Works with any camera.** Limelight and PhotonVision are built in, and any other camera works if it can give angles or field positions. Setup is a few lines.
- Everything else works like normal PathPlanner.

## Download the editor

Go to [Releases](../../releases/latest) and download the file for your laptop:

| Laptop | File | How to run |
| --- | --- | --- |
| Windows | `...-Windows-setup.exe` | Run the installer. A portable `.zip` is also available. |
| macOS | `...-macOS.zip` | Unzip and drag to Applications. The app is unsigned, so the first time, right-click it and choose **Open**. |
| Linux | `...-Linux.tar.gz` | Extract it and run `./pathplanner`. |

It installs separately from regular PathPlanner, so you can keep both.

## Add the robot library (Java)

Your robot code needs the matching library for backup paths to run on the robot.

1. Delete any `PathplannerLib*.json` (such as `PathplannerLib-2026.1.2.json`) from your robot project's `vendordeps` folder. This library replaces it.
2. In WPILib VS Code, open the command palette and run **WPILib: Manage Vendor Libraries → Install new libraries (online)**.
3. Paste:
   ```
   https://aarya470.github.io/pathplanner-backup-reactive-autos-paths/PathplannerLib-BackupPaths.json
   ```

Your existing PathPlanner code keeps working: the classes are still in `com.pathplanner.lib`. Java only for now.

## Set up cameras

Add this to your `RobotContainer`, after `AutoBuilder.configure(...)` and before building any autos:

```java
import com.pathplanner.lib.BackupPaths;
import com.pathplanner.lib.detection.Camera;

// 1. How to find and stop the robot: your drivetrain's pose method and a method that stops it
BackupPaths.setup(drivetrain::getPose, drivetrain::stop);

// 2. A camera that sees robots, for "Robot in the way" regions
BackupPaths.watchForRobots(
    Camera.limelight("limelight-front")
        .mountedAt(0.30, 0.0, 0.75, 15, 0)   // forward m, left m, height m, tilt down deg, facing deg
        .redClasses(1).blueClasses(2));      // which model classes are red and blue robots

// 3. A camera that sees game pieces, plus your intake, for "Game piece" regions
BackupPaths.pickUpGamePieces(
    Camera.limelight("limelight-intake").mountedAt(-0.35, 0.0, 0.45, 25, 180),
    intake::intakeCommand,   // the command that runs the intake
    intake::hasPiece,        // beam break, or null
    intake);                 // the intake subsystem
BackupPaths.intakeIsOnBack(); // only if it is
```

Steps 2 and 3 are each optional. Call them before `AutoBuilder.buildAutoChooser()` or any `new PathPlannerAuto(...)`: regions in autos built earlier are ignored (the Driver Station shows a warning).

Measure `mountedAt` from the robot's center on the floor: forward and left in meters (negative = behind or right), the lens height, how far the camera is tilted down, and which way it faces (0 forward, 90 left, 180 back, -90 right). `examples/backup-paths-java/camera_check.py` works out the tilt from one tape measurement and checks distances.

Which detections count: with `redClasses`/`blueClasses` set, only those classes count as robots. Without them, every class counts unless you use `onlyClasses(...)`. Use a separate `Camera` object for robots and for game pieces, each with its own classes. Two objects can read the same physical camera.

### Pick your camera

| Camera | Make it with | Needs |
| --- | --- | --- |
| Limelight (neural detector pipeline) | `Camera.limelight("limelight-name")` | nothing extra |
| PhotonVision (object detection) | `Camera.photon("CameraName")` | the [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html) vendordep, same version as PhotonVision on the coprocessor. Calibrate the camera for better distances. |
| Anything that gives angles | `Camera.fromAngles(() -> List.of(new CameraTarget(classId, tx, ty)))` | tx = degrees right of center, ty = degrees above center. Imports: `com.pathplanner.lib.detection.CameraTarget`, `java.util.List` |
| Anything that gives field positions | `Camera.fromFieldPositions(() -> listOfTranslation2d)` | no `mountedAt` needed |

Optional settings on any camera: `onlyClasses(...)` to ignore other model classes, `targetHeight(m)` if your robots or pieces are much taller or shorter than usual (defaults: 0.4 m for robots, 0.08 m for pieces), `maxRange(m)` (8 m), `maxAge(s)` (0.25 s), and `minConfidence(c)` (0.5, PhotonVision only; set Limelight's threshold on its web page).

### Use a camera anywhere

| Call | Gives |
| --- | --- |
| `camera.seesSomething()` | `true` if it sees anything |
| `camera.seesSomethingWithin(1.5)` | `true` if anything is within 1.5 m of the robot |
| `camera.getClosest()` | an `Optional` with the closest sighting, e.g. `camera.getClosest().ifPresent(s -> s.distanceMeters())`. A sighting has `fieldPosition()`, `distanceMeters()`, `classId()`, `color()` |
| `camera.getSightings()` | everything it sees |
| `SmartDashboard.putData("Front camera", camera)` | its status (`OK`, `Wrong class ID 3`, `Not mounted`, ...), count and closest distance |

Optional switches, for example from dashboard choosers: `BackupPaths.setSwitches(allowDivert, allowWait, allowPickups)`, each a `BooleanSupplier` (null = always on).

The full example is in [`examples/backup-paths-java`](examples/backup-paths-java), also attached to each release as `...-ExampleRobot.zip`. Code written for 1.2 and 1.3 (`PhotonRobotDetector`, `ObstacleSettings.configure`, ...) still works.

## Making a new release (maintainers)

```
git tag v1.0.1
git push origin v1.0.1
```

GitHub Actions builds Windows, macOS and Linux, publishes the vendordep to GitHub Pages, and creates the release.

## License and credit

PathPlanner is by Michael Jansen and contributors, under the MIT License (see `LICENSE`). This fork keeps that license.

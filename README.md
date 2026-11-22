# PathPlanner Backup Paths

A fork of [PathPlanner](https://github.com/mjansen4857/pathplanner) for FRC that adds **backup paths**: regions on a path that watch for robots or game pieces and change what your robot does in autonomous, on the fly.

Made by Aaryan Roy on FRC Team 1089. This is not an official PathPlanner release and is not affiliated with the PathPlanner project.

## What it adds

- **Robot in the way regions.** Draw a region on a path. If a robot is detected there, your robot waits, diverts onto a backup path you draw from the region edge, or continues.
- **Game piece regions.** If a game piece is seen while the robot is in the region, the robot drives to it with the intake running, then rejoins the main path or follows a divert path. Choose which pieces count: anything within a max distance of the robot, or anything inside a 4-corner **pickup zone** you drag on the field.
- **Both alliances are detected.** Teammates and opponents can both trigger a region.
- **Robot detection built in.** `PhotonRobotDetector` reads a PhotonVision object detection camera and tells you if a robot is ahead, how far and in which direction. Use it in autos, teleop or any command.
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

1. Delete `vendordeps/PathplannerLib.json` from your robot project if it's there. This library replaces it.
2. In WPILib VS Code, open the command palette and run **WPILib: Manage Vendor Libraries → Install new libraries (online)**.
3. Paste:
   ```
   https://aarya470.github.io/pathplanner-backup-reactive-autos-paths/PathplannerLib-BackupPaths.json
   ```

Your existing PathPlanner code keeps working: the classes are still in `com.pathplanner.lib`. Java only for now.

## Detect robots (PhotonVision)

1. Also install the [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html) vendordep.
2. In PhotonVision, run an object detection model that finds robots, and calibrate the camera at the resolution the detection pipeline uses. Nothing is reported until it is calibrated.
3. In your robot code, make the detector once and give it to the backup paths:

```java
PhotonRobotDetector detector = new PhotonRobotDetector(
    "DetectionCam",                                   // camera name in PhotonVision
    new Transform3d(                                  // where the camera sits on the robot
        new Translation3d(Units.inchesToMeters(-16), 0, Units.inchesToMeters(5)),
        new Rotation3d(0, 0, Math.PI)),               // yaw: which way it points; pitch: tilt down
    () -> drivetrain.getPose(),
    1, 2);                                            // the model's class numbers for robots

ObstacleSettings.configure(detector, () -> drivetrain.getPose(), () -> drivetrain.stop());
```

Call `configure` before building your autos. Every path with robot regions then uses the detector.

Use the detector anywhere else too:

| Call | Gives |
| --- | --- |
| `detector.isRobotAhead(1.5)` | `true` if a robot is in front of the camera within 1.5 m |
| `detector.isRobotSeen()` | `true` if any robot is seen |
| `detector.getClosestRobot()` / `getClosestRobotAhead()` | the nearest robot, if any |
| `robot.getDistanceMeters()` | distance from your robot's center |
| `robot.getBearingDegrees()` | direction from your robot's front (0 ahead, + left) |
| `robot.getCameraBearingDegrees()` | direction from the camera's view |
| `robot.getFieldPosition()` | where it is on the field |
| `detector.getStatus()` | `OK`, or why boxes were dropped (wrong class ID, not calibrated, ...) |

Optional settings: `setMinConfidence` (default 0.5), `setMaxAgeSeconds` (0.25), `setAheadHalfAngleDegrees` (30), and `setAllianceClassIds(red, blue)` if your model has separate red and blue robot classes, so regions can tell teammates from opponents.

Another camera? Implement `RobotDetector` (`getRobots`, `getAheadHalfAngleDegrees`, `getMaxAgeSeconds`) and pass it to `ObstacleSettings.configure` the same way.

The full example is in [`examples/backup-paths-java`](examples/backup-paths-java), also attached to each release as `...-ExampleRobot.zip`. Game piece pickups still take a `GamePieceSource` through `GamePieceSettings.configure`.

## Making a new release (maintainers)

```
git tag v1.0.1
git push origin v1.0.1
```

GitHub Actions builds Windows, macOS and Linux, publishes the vendordep to GitHub Pages, and creates the release.

## License and credit

PathPlanner is by Michael Jansen and contributors, under the MIT License (see `LICENSE`). This fork keeps that license.

# PathPlanner Backup Paths

A fork of the [PathPlanner](https://github.com/mjansen4857/pathplanner) editor for FRC that adds **regions**: parts of a path that watch for robots or game pieces and change what your robot does in autonomous, on the fly.

Made by Aaryan Roy on FRC Team 1089 Mercury. This is not an official PathPlanner release and is not affiliated with the PathPlanner project.

## The three pieces

| Piece | What it is | Where it runs |
| --- | --- | --- |
| **PathPlanner Backup Paths** (this app) | The path editor: paths, autos, and the regions and pickup zones on them | Your laptop |
| **Reactive Autonomous Pathing** (in `reactive-pathing/`) | Makes autos react to the regions you drew | The robot |
| **[Robot Detection](https://github.com/Aarya470/FRC-Robot-Vision)** | The camera library: sees robots and game pieces with a Limelight or PhotonVision camera | The robot |

Your robot code uses the **official PathplannerLib**, like every other team. The other two libraries are small add-ons. Robot Detection works on its own too, with no PathPlanner at all.

## What regions do

- **Robot in the way.** If a robot is seen in the region, your robot waits, diverts onto a backup path you drew, or keeps going.
- **Game piece.** If a game piece is seen inside the region's orange **pickup zone**, the robot drives to it with the intake running, then goes back to the path (or diverts). Drag the zone's 4 corners on the field to shape it.
- **Teammates and opponents.** A region can count any robot, only opponents, or only teammates.
- Everything else works like normal PathPlanner.

## Download the editor

Go to [Releases](../../releases/latest) and download the file for your laptop:

| Laptop | File | How to run |
| --- | --- | --- |
| Windows | `...-Windows-setup.exe` | Run the installer. A portable `.zip` is also available. |
| macOS | `...-macOS.zip` | Unzip and drag to Applications. The app is unsigned, so the first time, right-click it and choose **Open**. |
| Linux | `...-Linux.tar.gz` | Extract it and run `./pathplanner`. |

It installs separately from regular PathPlanner, so you can keep both. Edit your paths with this app: if the official PathPlanner app saves a path, it drops the regions.

## Set up the robot code (Java)

### 1. Install libraries

In WPILib VS Code, run **WPILib: Manage Vendor Libraries → Install new libraries (online)** for each:

```
https://aarya470.github.io/FRC-Robot-Vision/RobotDetection.json
https://aarya470.github.io/pathplanner-backup-reactive-autos-paths/ReactiveAutonomousPathing.json
```

For PhotonVision cameras, also install [PhotonLib](https://docs.photonvision.org/en/latest/docs/programming/photonlib/adding-vendordep.html), with the same version as PhotonVision on your coprocessor.

### 2. Swap one line in your drivetrain

Where you call `AutoBuilder.configure(...)`, call `ReactivePathing.configure(...)` instead, **with the same arguments**:

```java
import com.mercury1089.reactivepathing.ReactivePathing;

ReactivePathing.configure(
    this::getPose,
    this::resetPose,
    this::getSpeeds,
    (speeds, feedforwards) -> driveRobotRelative(speeds),
    new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
    config,
    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    this);
```

Don't also keep your old `AutoBuilder.configure(...)` call: PathPlanner can only be configured once, and `ReactivePathing.configure(...)` does it for you. Autos, `AutoBuilder.buildAutoChooser()`, named commands and event markers all work as before.

### 3. Give it your cameras and intake

In `RobotContainer`, before building autos:

```java
import com.mercury1089.detection.Camera;

Camera frontCamera = Camera.limelight("limelight-front")
    .mountedAt(0.30, 0.0, 0.75, 15, 0)   // forward m, left m, height m, tilt down degrees, facing degrees
    .redRobotClasses(1)
    .blueRobotClasses(2);

Camera intakeCamera = Camera.limelight("limelight-intake")
    .mountedAt(-0.35, 0.0, 0.45, 25, 180)
    .gamePieceClasses(0);

ReactivePathing.watchForRobots(frontCamera);
ReactivePathing.pickUpGamePieces(
    intakeCamera,
    intake::intakeCommand,   // the command that runs your intake
    intake::hasPiece);       // your beam break, or null
ReactivePathing.intakeIsOnBack();   // only if it is
```

During a pickup, your intake command runs while the robot heads for the piece and is stopped when it has it (or gives up). Autos that use the intake elsewhere (named commands, markers, parallel groups) work fine. If some other command outside the auto is using a subsystem your intake command needs, the intake isn't started, so that command isn't cancelled, and you get a warning.

Each camera needs its class numbers set (`redRobotClasses`, `gamePieceClasses`, ...). A camera with none sees nothing, and you'll get a warning.

Each part is optional: without a robot camera, robot regions just follow the path, and without a game piece camera, game piece regions just follow the path. One camera can do both, if its model sees both. See the [Robot Detection README](https://github.com/Aarya470/FRC-Robot-Vision) for every camera option, and for using cameras outside of autos.

### Other things you can use

| Call | Does |
| --- | --- |
| `ReactivePathing.pathfindToPose(pose, constraints)` | Drives to a pose. Use it instead of `AutoBuilder.pathfindToPose(...)`, which doesn't work after `ReactivePathing.configure(...)` |
| `ReactivePathing.pathfindToPoseFlipped(bluePose, constraints)` | The same, for a pose on the blue side, flipped when you are red. Use it instead of `AutoBuilder.pathfindToPoseFlipped(...)` |
| `ReactivePathing.pathfindThenFollowPath(path, constraints)` | Drives to a path's start, then follows it. Use it instead of `AutoBuilder.pathfindThenFollowPath(...)` |

While a path with regions runs, the dashboard shows `Reactive Pathing/Step`: following the path, waiting for a robot, picking up, going back to the path, or diverting.

Good to know:

- When the robot leaves the path (to pick up, or to wait and resume), it follows the rest of the path from where it joins. Event markers on the part it skipped don't run.
- The first pathfinding of a robot program is slow while Java warms up. Schedule `PathfindingCommand.warmupCommand()` in `robotInit()` so it isn't your auto.
- Choreo paths and paths made in code have no regions, so they run like normal PathPlanner paths.

The full example is in [`examples/backup-paths-java`](examples/backup-paths-java), also attached to each release as `...-ExampleRobot.zip`.

## Upgrading from 1.x

1.x used a modified PathplannerLib (`PathplannerLib-BackupPaths`). 2.0 uses the official one, plus two small libraries.

1. Delete `vendordeps/PathplannerLib-BackupPaths.json`.
2. Install the three libraries above.
3. Change `AutoBuilder.configure(...)` to `ReactivePathing.configure(...)`, with the same arguments. Don't keep the old call.
4. Delete `BackupPaths.setup(...)`. `configure(...)` does that job now.
5. Change `BackupPaths.watchForRobots(...)`, `pickUpGamePieces(camera, intakeCommand, hasPiece)` and `intakeIsOnBack()` to `ReactivePathing.` with the same name. `setSwitches(...)` and `overrideDivertPath(...)` are gone.
6. Change the pathfinding calls: `AutoBuilder.pathfindToPose(...)`, `pathfindToPoseFlipped(...)` and `pathfindThenFollowPath(...)` become `ReactivePathing.` with the same name.
7. Update the imports:
   ```java
   import com.mercury1089.reactivepathing.ReactivePathing;   // was com.pathplanner.lib.BackupPaths
   import com.mercury1089.detection.Camera;                   // was com.pathplanner.lib.detection.Camera
   import com.mercury1089.detection.Sighting;                 // was com.pathplanner.lib.detection.Sighting
   ```
8. Update the cameras:

   | 1.x | Now |
   | --- | --- |
   | `Camera.photon(name)` | `Camera.photonVision(name)` |
   | `Camera.fromFieldPositions(positions)` | `Camera.simulated(robotPositions, gamePiecePositions)` |
   | `Camera.fromAngles(...)` | Removed: use a Limelight or PhotonVision |
   | `redClasses(...)`, `blueClasses(...)` | `redRobotClasses(...)`, `blueRobotClasses(...)` |
   | `onlyClasses(...)` | `robotClasses(...)` or `gamePieceClasses(...)` |
   | `targetHeight(m)`, `setDefaultTargetHeight(m)` | `robotHeight(m)` and `gamePieceHeight(m)` |
   | `maxRange(m)` | `maxDistance(m)` |
   | `getClosest()` | `getClosestRobot()` or `getClosestGamePiece()`, which give a `Sighting` or `null` |
   | `getObstacles()` gave `List<Obstacle>` | `getRobots()` gives `List<Sighting>` |
   | `getGamePieces()` gave `List<GamePiece>` | `getGamePieces()` gives `List<Sighting>` |
   | `sighting.fieldPosition()`, `distanceMeters()`, `classId()`, `color()`, `timestampSeconds()` | `getFieldPosition()`, `getDistanceMeters()`, `getClassId()`, `getColor()`, `getTimestampSeconds()` |
   | `AllianceColor.RED` / `BLUE` | `Sighting.Color.RED` / `BLUE` / `UNKNOWN` |
   | `PhotonRobotDetector`, `PhotonGamePieceDetector`, `RobotDetector` | `Camera.photonVision(...)` with `robotClasses(...)` / `gamePieceClasses(...)` |

9. **List your classes.** In 1.x, a camera with no classes counted everything. Now it sees nothing. Add `robotClasses(...)` (or red/blue) and `gamePieceClasses(...)`.

Your paths and autos don't change.

## Upgrading from 2.0.0

The camera library was renamed from Mercury Vision to **Robot Detection** and made simpler.

1. Delete `vendordeps/MercuryVision.json` and install `https://aarya470.github.io/FRC-Robot-Vision/RobotDetection.json`. Update Reactive Autonomous Pathing to 2.1.0.
2. Change the imports from `com.mercury1089.vision` to `com.mercury1089.detection`.
3. `TeamColor` is now `Sighting.Color`, and `ThingType` is now `Sighting.Type`.
4. `VisionCommands.waitUntil...(camera, ...)` is now `camera.waitUntil...(...)`, and `VisionCommands.driveToClosestGamePiece(...)` is now `new DriveToGamePieceCommand(...)` with the same arguments.
5. `ReactivePathing.pickUpGamePieces(...)` no longer takes your intake subsystem at the end: remove it.
6. `setSwitches(...)`, `overrideDivertPath(...)`, `followPathNormally(...)` and the `LinearVelocity` versions of `pathfindToPose` and `pathfindToPoseFlipped` were removed. Give the end speed in meters per second instead.

See the [Robot Detection changelog](https://github.com/Aarya470/FRC-Robot-Vision/blob/main/CHANGELOG.md) for the rest.

## Making a new release (maintainers)

1. Add a `## X.Y.Z` section to `CHANGELOG.md`.
2. Tag and push:
   ```
   git tag vX.Y.Z
   git push origin vX.Y.Z
   ```

GitHub Actions builds the app for Windows, macOS and Linux, tests and publishes Reactive Autonomous Pathing to GitHub Pages, and creates the release.

### Releasing with a new Robot Detection version

Reactive Autonomous Pathing is built against Robot Detection, so release Robot Detection first:

1. In the `FRC-Robot-Vision` repo, push its new tag (like `v2.0.0`) and wait for its release to finish.
2. Check that https://aarya470.github.io/FRC-Robot-Vision/RobotDetection.json opens and shows that version.
3. Set `robotDetectionVersion` in `reactive-pathing/build.gradle` to it, then release this repo.

This repo's CI builds Robot Detection from that tag, so the tag must exist before the checks pass.

### Running the library tests on your computer

`cd reactive-pathing` then `./gradlew build`. The tests run a simulated robot with WPILib's native libraries, which Gradle downloads for Linux, Windows or macOS automatically.

Before a new Robot Detection version is on GitHub Pages, build it into your computer's Maven folder first: in the FRC-Robot-Vision repo, run `./gradlew publishToMavenLocal -PreleaseVersion=2.0.0`.

## License and credit

PathPlanner is by Michael Jansen and contributors, under the MIT License (see `LICENSE`). This fork keeps that license.

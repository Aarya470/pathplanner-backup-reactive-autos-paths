# Reactive Autonomous Pathing

The robot-side library for [PathPlanner Backup Paths](../README.md). It makes PathPlanner autos react to the regions you draw in the app: waiting for or diverting around robots, and picking up game pieces.

It builds on the official [PathplannerLib](https://pathplanner.dev) and uses cameras from [Mercury Vision](https://github.com/Aarya470/FRC-Robot-Vision). Setup is in the main [README](../README.md#set-up-the-robot-code-java).

## How the code is laid out

| File | What it does |
| --- | --- |
| `ReactivePathing.java` | The setup calls (`configure`, `watchForRobots`, `pickUpGamePieces`, ...). It gives PathPlanner the command for each path in an auto. |
| `ReactivePathCommand.java` | Follows a path with regions, one step at a time: following the path, waiting for a robot, picking up, going back to the path, diverting. |
| `RobotRegionWatcher.java` | The rules for a "robot in the way" region: when to stop, resume or divert |
| `RegionFile.java`, `PathFileReader.java` | Read the regions the app saves in `.path` files. PathplannerLib ignores this part of the file. |
| `PathCutter.java` | Makes "the rest of a path" from a point partway along it, keeping markers, rotation targets and zones |
| `PathMath.java` | Small helpers: flipping to red, mirroring, the closest point, the rotation along a path, inside a zone |
| `Region.java` and the enums | The data for one region |

## Build and test

```
./gradlew build
```

The tests include whole autos run on a simulated robot (`SimulatedAutoTest.java`): waiting, diverting, picking up and going back to the path, using PathplannerLib's real path follower.

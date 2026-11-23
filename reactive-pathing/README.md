# Reactive Autonomous Pathing

The robot-side library for [PathPlanner Backup Paths](../README.md). It makes PathPlanner autos react to the regions you draw in the app: waiting for or diverting around robots, and picking up game pieces.

It builds on the official [PathplannerLib](https://pathplanner.dev) and uses cameras from [Robot Detection](https://github.com/Aarya470/FRC-Robot-Vision). Setup is in the main [README](../README.md#set-up-the-robot-code-java).

## How the code is laid out

| File | What it does |
| --- | --- |
| `ReactivePathing.java` | The setup calls (`configure`, `watchForRobots`, `pickUpGamePieces`, the pathfinding commands). It gives PathPlanner the command for each path in an auto. |
| `ReactivePathCommand.java` | Follows a path with regions, one step at a time: following the path, waiting for a robot, picking up, going back to the path, diverting. |
| `Region.java` | One region, and reading the regions the app saves in `.path` files (PathplannerLib ignores that part of the file) |
| `PathTools.java` | Path helpers: "the rest of a path" from a point partway along it, mirroring, the rotation along a path, inside a zone |

## Build and test

```
./gradlew build
```

The tests include whole autos run on a simulated robot (`SimulatedAutoTest.java`): waiting, diverting, picking up and going back to the path, using PathplannerLib's real path follower.

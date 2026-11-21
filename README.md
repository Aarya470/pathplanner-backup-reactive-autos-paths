# PathPlanner Backup Paths

A fork of [PathPlanner](https://github.com/mjansen4857/pathplanner) for FRC that adds **backup paths**: regions on a path that watch for robots or game pieces and change what your robot does in autonomous, on the fly.

Made by Aaryan Roy on FRC Team 1089. This is not an official PathPlanner release and is not affiliated with the PathPlanner project.

## What it adds

- **Robot in the way regions.** Draw a region on a path. If a robot is detected there, your robot waits, diverts onto a backup path you draw from the region edge, or continues.
- **Game piece regions.** If a game piece is seen while the robot is in the region, the robot drives to it with the intake running, then rejoins the main path or follows a divert path. Choose which pieces count: anything within a max distance of the robot, or anything inside a 4-corner **pickup zone** you drag on the field.
- **Both alliances are detected.** Teammates and opponents can both trigger a region.
- **Dashboard toggles** (Elastic) to turn divert, wait and pickup on or off without redeploying.
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

## Connect your camera (example robot project)

The library decides what to do in each region, but your robot code has to tell it where robots and game pieces are. The example in [`examples/backup-paths-java`](examples/backup-paths-java) (also attached to each release as `...-ExampleRobot.zip`) shows how, with a PhotonVision object detection camera. It runs in the simulator with no camera, using a fake robot and game piece you place from the dashboard.

## Making a new release (maintainers)

```
git tag v1.0.1
git push origin v1.0.1
```

GitHub Actions builds Windows, macOS and Linux, publishes the vendordep to GitHub Pages, and creates the release.

## License and credit

PathPlanner is by Michael Jansen and contributors, under the MIT License (see `LICENSE`). This fork keeps that license.

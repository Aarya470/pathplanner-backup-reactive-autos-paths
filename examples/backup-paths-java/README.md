# Example robot project: PathPlanner Backup Paths

A small swerve robot (it runs in the simulator) that shows the whole setup:

- **Official PathplannerLib** for paths and autos
- **Mercury Vision** for the cameras
- **Reactive Autonomous Pathing** to make the autos react to the regions drawn in the app

The paths:

- `Score.path` has a **robot in the way** region. When the robot camera sees a robot blocking it, the robot stops at the end of the region, waits 1 s, then goes on.
- `Pickup.path` has a **game piece** region with an orange pickup zone. When the piece camera sees a piece inside the zone, the robot drives to it, runs the intake, and goes back to the path.

## Try it

1. Open this folder in WPILib VS Code (2026).
2. Run **WPILib: Simulate Robot Code**, or deploy it to a robot after setting your team number in `.wpilib/wpilib_preferences.json`.
3. Open the paths in **PathPlanner Backup Paths**. Click a region on the right to see its part of the path in yellow, and drag the 4 orange dots to reshape a pickup zone.
4. Run `Example Auto` from the dashboard.

The dashboard shows:

- **Robot Camera** and **Piece Camera**: each camera's status (`OK`, `Ignored class 3`, `Not mounted`, ...), what it sees, and the closest distances
- **Reactive Pathing/Step**: what the robot is doing on a path with regions

## Where everything is

| What | Where |
| --- | --- |
| Path following setup: `ReactivePathing.configure(...)`, the same arguments as `AutoBuilder.configure(...)` | `subsystems/SwerveSubsystem.java` |
| The cameras | the top of `RobotContainer.java` |
| Connecting the cameras and intake to the regions | `configureRegions()` in `RobotContainer.java` |
| A camera used outside of autos (`robotNearby(1.5)`) | `configureBindings()` in `RobotContainer.java` |

## Put it in your own robot code

1. Install the vendor libraries (WPILib VS Code: **Manage Vendor Libraries → Install new libraries (online)**):
   ```
   https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib.json
   https://aarya470.github.io/FRC-Robot-Vision/MercuryVision.json
   https://aarya470.github.io/pathplanner-backup-reactive-autos-paths/ReactiveAutonomousPathing.json
   ```
   For PhotonVision cameras, also install PhotonLib. Its version must match PhotonVision on your coprocessor (this example uses `v2026.3.4`). For Limelights, you don't need it.
2. Change `AutoBuilder.configure(...)` to `ReactivePathing.configure(...)`. Keep the arguments.
3. Copy the camera fields and `configureRegions()` from `RobotContainer.java`, and call `configureRegions()` before `AutoBuilder.buildAutoChooser()`.
4. Measure your cameras for `mountedAt(forward, left, height, tiltDown, facing)`, from the robot's center on the floor. Mercury Vision's `tools/camera_check.py` helps.
5. Set the class numbers your model uses: `redRobotClasses`, `blueRobotClasses` (or `robotClasses`) and `gamePieceClasses`.
6. Put in your intake's command, beam break (or `null`) and subsystem, for example `ReactivePathing.pickUpGamePieces(pieceCamera, intake::intakeCommand, intake::hasPiece, intake)`. The subsystem matters: it is how the library tells if your auto is already running the intake, so it doesn't start it twice. Remove `intakeIsOnBack()` if your intake is on the front.

Using a Limelight? Change `Camera.photonVision("...")` to `Camera.limelight("limelight-name")`. Nothing else changes.

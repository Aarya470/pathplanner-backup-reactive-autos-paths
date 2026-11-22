package com.pathplanner.lib.detection;

import com.pathplanner.lib.BackupPaths;
import com.pathplanner.lib.obstacle.AllianceColor;
import com.pathplanner.lib.obstacle.GamePiece;
import com.pathplanner.lib.obstacle.GamePieceSource;
import com.pathplanner.lib.obstacle.Obstacle;
import com.pathplanner.lib.obstacle.ObstacleSettings;
import com.pathplanner.lib.obstacle.ObstacleSource;
import com.pathplanner.lib.obstacle.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

/**
 * An object detection camera, for any brand. Make one with a factory method, say where it sits on
 * the robot, then hand it to {@link BackupPaths}:
 *
 * <pre>{@code
 * Camera front = Camera.limelight("limelight-front")
 *     .mountedAt(0.30, 0.0, 0.75, 15, 0);   // forward m, left m, height m, tilt down deg, facing deg
 * BackupPaths.watchForRobots(front);
 * }</pre>
 *
 * <ul>
 *   <li>{@link #limelight(String)}: a Limelight running a neural detector pipeline. No extra library
 *       needed.
 *   <li>{@link #photon(String)}: a PhotonVision object detection camera. Needs the photonlib
 *       vendordep.
 *   <li>{@link #fromAngles(Supplier)}: any other camera that can tell you the left-right and up-down
 *       angle to each thing it sees.
 *   <li>{@link #fromFieldPositions(Supplier)}: any other system that already gives field positions.
 * </ul>
 *
 * <p>You can also read a camera anywhere in your robot code, for example {@code
 * front.seesSomethingWithin(2.0)}, and put it on a dashboard with {@code
 * SmartDashboard.putData("Front camera", front)} to see its status while setting up.
 */
public abstract class Camera implements ObstacleSource, GamePieceSource, Sendable {
  /** What a camera saw in one frame. Either angles or field positions, never both. */
  protected record Reading(
      List<CameraTarget> angles,
      List<Placed> placed,
      double timestampSeconds,
      String problem,
      String noteIfEmpty) {
    protected Reading(
        List<CameraTarget> angles, List<Placed> placed, double timestampSeconds, String problem) {
      this(angles, placed, timestampSeconds, problem, null);
    }

    static Reading problem(String why) {
      return new Reading(List.of(), List.of(), Timer.getFPGATimestamp(), why);
    }
  }

  /** Something already placed on the field by the camera itself, with its class number. */
  protected record Placed(Translation2d fieldPosition, int classId) {}

  // Where the camera sits
  double forwardMeters = 0.0;
  double leftMeters = 0.0;
  double heightMeters = 0.5;
  double tiltDownDegrees = 0.0;
  double facingDegrees = 0.0;
  boolean mounted = false;

  private Double targetHeightMeters = null;
  private double defaultTargetHeightMeters = 0.0;
  private double maxRangeMeters = 8.0;
  private double maxAgeSeconds = 0.25;
  private double minConfidence = 0.5;
  private final Set<Integer> onlyClasses = new HashSet<>();
  private final Set<Integer> redClasses = new HashSet<>();
  private final Set<Integer> blueClasses = new HashSet<>();

  private List<Sighting> sightings = new ArrayList<>();
  private String status = "No frames yet";
  private double lastPollTime = -1;

  /**
   * A Limelight running a neural detector pipeline. Reads Limelight's own NetworkTables output, so
   * LimelightHelpers is not needed.
   *
   * @param name the Limelight's name, such as "limelight" or "limelight-front"
   */
  public static Camera limelight(String name) {
    return LimelightCamera.fromNetworkTables(name);
  }

  /**
   * A PhotonVision object detection camera. Needs the photonlib vendordep, with the same version as
   * PhotonVision on your coprocessor. If the camera is calibrated in PhotonVision, the calibration
   * is used for better distances.
   *
   * @param name the camera's name in PhotonVision, exactly as typed there
   */
  public static Camera photon(String name) {
    return new PhotonVisionCamera(name);
  }

  /**
   * Any camera that can tell you the angles to what it sees. Give a function that returns
   * everything the camera sees right now. Return an empty list when it sees nothing.
   *
   * <pre>{@code
   * Camera cam = Camera.fromAngles(() -> List.of(new CameraTarget(0, myTx(), myTy())))
   *     .mountedAt(0.3, 0, 0.6, 20, 0);
   * }</pre>
   */
  public static Camera fromAngles(Supplier<List<CameraTarget>> targets) {
    return new Camera() {
      @Override
      protected Reading read() {
        List<CameraTarget> list = targets.get();
        return new Reading(
            list == null ? List.of() : list, List.of(), Timer.getFPGATimestamp(), null);
      }
    };
  }

  /**
   * Any system that already knows where things are on the field, such as your own vision code.
   * Give a function that returns their field positions right now. No mounting info is needed.
   */
  public static Camera fromFieldPositions(Supplier<List<Translation2d>> positions) {
    Camera camera = new Camera() {
      @Override
      protected Reading read() {
        List<Placed> placed = new ArrayList<>();
        List<Translation2d> list = positions.get();
        if (list != null) {
          for (Translation2d p : list) {
            placed.add(new Placed(p, -1));
          }
        }
        return new Reading(List.of(), placed, Timer.getFPGATimestamp(), null);
      }
    };
    camera.mounted = true;
    return camera;
  }

  /**
   * Where the camera sits on the robot.
   *
   * @param forwardMeters how far in front of the robot's center (negative = behind)
   * @param leftMeters how far left of the robot's center (negative = right)
   * @param heightMeters how high the lens is above the floor
   * @param tiltDownDegrees how far the camera is tilted down (0 = level)
   * @param facingDegrees which way it points: 0 = forward, 90 = left, 180 = backward, -90 = right
   * @return this camera, so calls can be chained
   */
  public Camera mountedAt(
      double forwardMeters,
      double leftMeters,
      double heightMeters,
      double tiltDownDegrees,
      double facingDegrees) {
    this.forwardMeters = forwardMeters;
    this.leftMeters = leftMeters;
    this.heightMeters = heightMeters;
    this.tiltDownDegrees = tiltDownDegrees;
    this.facingDegrees = facingDegrees;
    this.mounted = true;
    onMountChanged();
    return this;
  }

  /** Called after {@link #mountedAt} changes the mount. */
  protected void onMountChanged() {}

  /** Only use these class numbers from the model. Everything else is ignored. */
  public Camera onlyClasses(int... classIds) {
    for (int id : classIds) onlyClasses.add(id);
    return this;
  }

  /**
   * These class numbers are red alliance robots, so regions can tell opponents from teammates.
   * Once red or blue classes are set, other classes are ignored unless listed in onlyClasses.
   */
  public Camera redClasses(int... classIds) {
    for (int id : classIds) redClasses.add(id);
    return this;
  }

  /**
   * These class numbers are blue alliance robots, so regions can tell opponents from teammates.
   * Once red or blue classes are set, other classes are ignored unless listed in onlyClasses.
   */
  public Camera blueClasses(int... classIds) {
    for (int id : classIds) blueClasses.add(id);
    return this;
  }

  /**
   * How high the middle of what the camera sees is above the floor. Defaults to 0.4 m for robots
   * and 0.08 m for game pieces, so you only need this if yours are very different.
   */
  public Camera targetHeight(double meters) {
    targetHeightMeters = meters;
    return this;
  }

  /** Ignore anything farther than this from the robot. Default 8 m. */
  public Camera maxRange(double meters) {
    maxRangeMeters = meters;
    return this;
  }

  /** Forget what the camera saw after this many seconds. Default 0.25 s. */
  public Camera maxAge(double seconds) {
    maxAgeSeconds = seconds;
    return this;
  }

  /**
   * Ignore detections below this confidence, 0 to 1. Default 0.5. Only cameras that report
   * confidence use it (PhotonVision). Limelight applies its own threshold, set in its web page.
   */
  public Camera minConfidence(double confidence) {
    minConfidence = confidence;
    return this;
  }

  /** @return the lowest confidence a detection needs */
  protected double getMinConfidence() {
    return minConfidence;
  }

  /** Used by BackupPaths: the target height for this use, unless you set one. */
  public void setDefaultTargetHeight(double meters) {
    defaultTargetHeightMeters = meters;
  }

  /** @return the target height used when none is set, for tests */
  public double getDefaultTargetHeightForTests() {
    return defaultTargetHeightMeters;
  }

  /** @return the most this camera keeps a sighting, in seconds */
  public double getMaxAgeSeconds() {
    return maxAgeSeconds;
  }

  /** Read the camera's latest frame. Called for you; subclasses for new camera types fill it in. */
  protected abstract Reading read();

  /** @return everything the camera sees right now, placed on the field */
  public List<Sighting> getSightings() {
    poll();
    double now = Timer.getFPGATimestamp();
    List<Sighting> fresh = new ArrayList<>();
    for (Sighting s : sightings) {
      if (now - s.timestampSeconds() <= maxAgeSeconds) fresh.add(s);
    }
    return fresh;
  }

  /** @return true if the camera sees anything right now */
  public boolean seesSomething() {
    return !getSightings().isEmpty();
  }

  /** @return the closest thing the camera sees, if any */
  public Optional<Sighting> getClosest() {
    Sighting closest = null;
    for (Sighting s : getSightings()) {
      if (closest == null || s.distanceMeters() < closest.distanceMeters()) closest = s;
    }
    return Optional.ofNullable(closest);
  }

  /** @return true if the camera sees anything within this many meters of the robot */
  public boolean seesSomethingWithin(double meters) {
    return getClosest().map(s -> s.distanceMeters() <= meters).orElse(false);
  }

  /**
   * @return "OK" or why nothing is coming through, such as "Not mounted: call mountedAt(...)" or
   *     "Wrong class ID 3". Shown on the dashboard.
   */
  public String getStatus() {
    poll();
    return status;
  }

  /** The last status, without reading a new frame */
  String getStatusWithoutPolling() {
    return status;
  }

  @Override
  public List<Obstacle> getObstacles() {
    List<Obstacle> out = new ArrayList<>();
    for (Sighting s : getSightings()) {
      out.add(
          new Obstacle(
              new Point(s.fieldPosition().getX(), s.fieldPosition().getY()),
              s.color(),
              s.timestampSeconds()));
    }
    return out;
  }

  @Override
  public List<GamePiece> getGamePieces() {
    List<GamePiece> out = new ArrayList<>();
    for (Sighting s : getSightings()) {
      out.add(
          new GamePiece(
              new Point(s.fieldPosition().getX(), s.fieldPosition().getY()),
              s.timestampSeconds()));
    }
    return out;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PathPlannerCamera");
    builder.addStringProperty("Status", this::getStatus, null);
    builder.addDoubleProperty("Seen", () -> getSightings().size(), null);
    builder.addDoubleProperty(
        "Closest m", () -> getClosest().map(Sighting::distanceMeters).orElse(-1.0), null);
  }

  private void poll() {
    double now = Timer.getFPGATimestamp();
    if (lastPollTime >= 0 && now - lastPollTime < 0.005) return;
    lastPollTime = now;

    Supplier<Pose2d> poseSupplier = robotPose();
    if (poseSupplier == null) {
      status = "Call BackupPaths.setup(...) first";
      sightings = new ArrayList<>();
      return;
    }
    Reading reading = read();
    if (reading == null) return; // no new frame, keep the last one until it ages out
    if (reading.problem() != null) {
      status = reading.problem();
      sightings = new ArrayList<>();
      return;
    }
    if (!reading.angles().isEmpty() && !mounted) {
      status = "Not mounted: call mountedAt(...)";
      sightings = new ArrayList<>();
      return;
    }
    sightings = place(reading, poseSupplier.get());
  }

  /** The robot pose from BackupPaths.setup, or from ObstacleSettings for older setups */
  static Supplier<Pose2d> robotPose() {
    Supplier<Pose2d> pose = BackupPaths.getPoseSupplier();
    return pose != null ? pose : ObstacleSettings.getPoseSupplier();
  }

  /** Turn one frame into field sightings, keeping the class filter and colors. */
  List<Sighting> place(Reading reading, Pose2d robot) {
    List<Sighting> out = new ArrayList<>();
    String note =
        reading.angles().isEmpty() && reading.placed().isEmpty()
            ? (reading.noteIfEmpty() != null ? reading.noteIfEmpty() : "Sees nothing")
            : "OK";
    double targetHeight =
        targetHeightMeters != null ? targetHeightMeters : defaultTargetHeightMeters;

    List<Placed> placed = new ArrayList<>(reading.placed());
    for (CameraTarget t : reading.angles()) {
      if (!classAllowed(t.classId())) {
        note = "Wrong class ID " + t.classId();
        continue;
      }
      Optional<Translation2d> rel =
          toRobotRelative(
              forwardMeters,
              leftMeters,
              heightMeters,
              tiltDownDegrees,
              facingDegrees,
              targetHeight,
              t.txDegrees(),
              t.tyDegrees());
      if (rel.isEmpty()) {
        note =
            Math.abs(heightMeters - targetHeight) < 0.05
                ? "Camera is level with targetHeight (" + targetHeight + " m): set targetHeight(...)"
                : "Ray never reaches the target height (check height and tilt)";
        continue;
      }
      placed.add(
          new Placed(
              robot.getTranslation().plus(rel.get().rotateBy(robot.getRotation())), t.classId()));
    }

    for (Placed p : placed) {
      if (p.classId() >= 0 && !classAllowed(p.classId())) {
        note = "Wrong class ID " + p.classId();
        continue;
      }
      double distance = p.fieldPosition().getDistance(robot.getTranslation());
      if (distance > maxRangeMeters) {
        note = "Farther than max range";
        continue;
      }
      out.add(
          new Sighting(
              p.fieldPosition(), distance, p.classId(), colorOf(p.classId()), reading.timestampSeconds()));
    }
    status = out.isEmpty() ? note : "OK";
    return out;
  }

  private boolean classAllowed(int classId) {
    if (!onlyClasses.isEmpty()) return onlyClasses.contains(classId);
    // With red or blue classes set, only those count
    if (!redClasses.isEmpty() || !blueClasses.isEmpty()) {
      return redClasses.contains(classId) || blueClasses.contains(classId);
    }
    return true;
  }

  private AllianceColor colorOf(int classId) {
    if (redClasses.contains(classId)) return AllianceColor.RED;
    if (blueClasses.contains(classId)) return AllianceColor.BLUE;
    return AllianceColor.UNKNOWN;
  }

  /**
   * Where a target seen at these angles is, relative to the robot's center (x forward, y left), by
   * following the ray from the lens until it reaches the target's height.
   *
   * @return empty if the ray never gets there (it points above the horizon, for example)
   */
  public static Optional<Translation2d> toRobotRelative(
      double forwardMeters,
      double leftMeters,
      double heightMeters,
      double tiltDownDegrees,
      double facingDegrees,
      double targetHeightMeters,
      double txDegrees,
      double tyDegrees) {
    // Ray in the camera's frame: x out of the lens, y left, z up
    double x = 1.0;
    double y = -Math.tan(Math.toRadians(txDegrees));
    double z = Math.tan(Math.toRadians(tyDegrees));

    // Tilt the camera down
    double p = Math.toRadians(tiltDownDegrees);
    double xt = x * Math.cos(p) + z * Math.sin(p);
    double zt = -x * Math.sin(p) + z * Math.cos(p);

    double drop = targetHeightMeters - heightMeters;
    if (Math.abs(zt) < 1e-9) return Optional.empty();
    double t = drop / zt;
    // Behind the lens, or so shallow it is practically at the horizon
    if (t <= 0 || Math.abs(zt) / Math.hypot(xt, y) < Math.tan(Math.toRadians(0.5))) {
      return Optional.empty();
    }

    Translation2d fromLens =
        new Translation2d(t * xt, t * y).rotateBy(Rotation2d.fromDegrees(facingDegrees));
    return Optional.of(new Translation2d(forwardMeters, leftMeters).plus(fromLens));
  }
}

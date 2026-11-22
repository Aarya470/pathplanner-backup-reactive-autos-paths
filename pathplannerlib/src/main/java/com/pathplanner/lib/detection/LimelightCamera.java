package com.pathplanner.lib.detection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * A Limelight running a neural detector pipeline. Reads the "rawdetections" array Limelight
 * publishes: 12 numbers per detection, starting with the class number, then the angles to the
 * middle of the box (txnc, tync). Made with {@link Camera#limelight(String)}.
 */
final class LimelightCamera extends Camera {
  /** Numbers per detection in "rawdetections" */
  static final int VALUES_PER_DETECTION = 12;

  /** Seconds without a new heartbeat before the Limelight counts as disconnected */
  static final double DISCONNECTED_AFTER_SEC = 0.5;

  private final String name;
  private final Supplier<double[]> rawDetections;
  private final DoubleSupplier latencyMs;
  private final DoubleSupplier heartbeat;

  private double lastHeartbeat = Double.NaN;
  private double lastHeartbeatTime = -1;

  LimelightCamera(
      String name,
      Supplier<double[]> rawDetections,
      DoubleSupplier latencyMs,
      DoubleSupplier heartbeat) {
    this.name = name;
    this.rawDetections = rawDetections;
    this.latencyMs = latencyMs;
    this.heartbeat = heartbeat;
  }

  static LimelightCamera fromNetworkTables(String name) {
    String table = name == null || name.isBlank() ? "limelight" : name;
    NetworkTable nt = NetworkTableInstance.getDefault().getTable(table);
    var raw = nt.getDoubleArrayTopic("rawdetections").subscribe(new double[0]);
    var tl = nt.getDoubleTopic("tl").subscribe(0.0);
    var cl = nt.getDoubleTopic("cl").subscribe(0.0);
    var hb = nt.getDoubleTopic("hb").subscribe(-1.0);
    return new LimelightCamera(table, raw::get, () -> tl.get() + cl.get(), hb::get);
  }

  @Override
  protected Reading read() {
    double now = Timer.getFPGATimestamp();
    double hb = heartbeat.getAsDouble();
    if (hb < 0) {
      return Reading.problem("No data from " + name + " (check the name and that it is on)");
    }
    if (hb != lastHeartbeat) {
      lastHeartbeat = hb;
      lastHeartbeatTime = now;
    } else if (lastHeartbeatTime < 0 || now - lastHeartbeatTime > DISCONNECTED_AFTER_SEC) {
      return Reading.problem("No data from " + name + " (check the name and that it is on)");
    } else {
      return null; // same frame as before
    }

    double[] raw = rawDetections.get();
    double timestamp = now - latencyMs.getAsDouble() / 1000.0;
    return new Reading(parse(raw), List.of(), timestamp, null);
  }

  /** Turn Limelight's rawdetections array into targets */
  static List<CameraTarget> parse(double[] raw) {
    List<CameraTarget> targets = new ArrayList<>();
    if (raw == null) return targets;
    for (int i = 0; i + VALUES_PER_DETECTION <= raw.length; i += VALUES_PER_DETECTION) {
      targets.add(new CameraTarget((int) raw[i], raw[i + 1], raw[i + 2]));
    }
    return targets;
  }
}

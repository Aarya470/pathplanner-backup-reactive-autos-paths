package frc.robot.sensors;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Settings for the detection camera, read from src/main/deploy/detection-camera.json so they can
 * change without a code change. Each season, only the "classes" map changes when the model does.
 */
public class DetectionCameraConfig {

    /** What a model class means to the robot */
    public enum Role {
        GAME_PIECE,
        ROBOT_RED,
        ROBOT_BLUE,
        /** A robot whose bumper color the model doesn't tell */
        ROBOT,
        IGNORE;

        static Role fromString(String name) {
            switch (name) {
                case "gamePiece": return GAME_PIECE;
                case "robotRed": return ROBOT_RED;
                case "robotBlue": return ROBOT_BLUE;
                case "robot": return ROBOT;
                case "ignore": return IGNORE;
                default:
                    throw new IllegalArgumentException(
                        "detection-camera.json: unknown class role \"" + name
                            + "\". Use gamePiece, robotRed, robotBlue, robot, or ignore.");
            }
        }
    }

    public final String cameraName;
    public final double heightMeters;
    public final double pitchDownRadians;
    public final double yawRadians;
    public final double forwardMeters;
    public final double leftMeters;
    /** True if fx, fy, cx, cy are in the file. Otherwise they come from PhotonVision's calibration. */
    public final boolean hasIntrinsics;
    public final double fx, fy, cx, cy;
    public final double minConfidence;
    public final double staleAfterSec;
    public final double intakeFacingDegrees;
    public final double intakeAfterArriveSec;
    public final double maxPickupSec;
    /** DIO port of the intake beam break that says the robot holds a piece, or -1 for none */
    public final int intakeBeamBreakPort;
    private final Map<Integer, Role> roles = new HashMap<>();

    private DetectionCameraConfig(Map<?, ?> json) {
        cameraName = (String) json.get("cameraName");
        if (cameraName == null) {
            throw new IllegalArgumentException("detection-camera.json is missing cameraName");
        }
        heightMeters = length(json, "height");
        pitchDownRadians = Math.toRadians(num(json, "pitchDownDegrees"));
        yawRadians = Math.toRadians(num(json, "yawDegrees"));
        forwardMeters = length(json, "forward");
        leftMeters = length(json, "left");

        hasIntrinsics = json.get("fx") instanceof Number;
        fx = hasIntrinsics ? num(json, "fx") : 0;
        fy = hasIntrinsics ? num(json, "fy") : 0;
        cx = hasIntrinsics ? num(json, "cx") : 0;
        cy = hasIntrinsics ? num(json, "cy") : 0;

        minConfidence = num(json, "minConfidence");
        staleAfterSec = num(json, "staleAfterSec");
        intakeFacingDegrees = num(json, "intakeFacingDegrees");
        intakeAfterArriveSec = num(json, "intakeAfterArriveSec");
        maxPickupSec = num(json, "maxPickupSec");
        Object port = json.get("intakeBeamBreakPort");
        intakeBeamBreakPort = port instanceof Number ? ((Number) port).intValue() : -1;

        Object classes = json.get("classes");
        if (!(classes instanceof Map)) {
            throw new IllegalArgumentException("detection-camera.json is missing the classes map");
        }
        for (Map.Entry<?, ?> e : ((Map<?, ?>) classes).entrySet()) {
            roles.put(Integer.parseInt(e.getKey().toString().trim()), Role.fromString(e.getValue().toString()));
        }
    }

    /** @return what a model class ID means, IGNORE if it isn't listed */
    public Role roleOf(int classId) {
        return roles.getOrDefault(classId, Role.IGNORE);
    }

    /**
     * Load the config from the deploy folder.
     *
     * @return the parsed settings
     * @throws Exception if the file is missing, isn't valid JSON, or is missing a field
     */
    public static DetectionCameraConfig load() throws Exception {
        Path file = Filesystem.getDeployDirectory().toPath().resolve("detection-camera.json");
        String text = Files.readString(file, StandardCharsets.UTF_8);
        Map<?, ?> json = (Map<?, ?>) new JSONParser().parse(text);
        return new DetectionCameraConfig(json);
    }

    /** A length given as name + "Inches" or name + "Meters", in meters. */
    private static double length(Map<?, ?> json, String name) {
        if (json.get(name + "Inches") instanceof Number) {
            return ((Number) json.get(name + "Inches")).doubleValue() * 0.0254;
        }
        return num(json, name + "Meters");
    }

    private static double num(Map<?, ?> json, String key) {
        Object value = json.get(key);
        if (!(value instanceof Number)) {
            throw new IllegalArgumentException("detection-camera.json is missing a number: " + key);
        }
        return ((Number) value).doubleValue();
    }
}

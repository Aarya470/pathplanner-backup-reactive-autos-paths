package com.pathplanner.lib.detection;

import com.pathplanner.lib.obstacle.AllianceColor;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Something a {@link Camera} saw, placed on the field.
 *
 * @param fieldPosition where it is on the field, in meters (blue alliance origin, like your
 *     robot's pose)
 * @param distanceMeters how far it is from the robot's center
 * @param classId the class number the detection model gave it, or -1 if the camera does not say
 * @param color red or blue if you told the camera which classes are which, otherwise UNKNOWN
 * @param timestampSeconds when the camera saw it (FPGA time)
 */
public record Sighting(
    Translation2d fieldPosition,
    double distanceMeters,
    int classId,
    AllianceColor color,
    double timestampSeconds) {}

package com.pathplanner.lib.detection;

/**
 * One thing a camera sees, as angles from the middle of the image. This is what Limelight's tx and
 * ty give you, and what most other cameras can give you too.
 *
 * @param classId the class number the detection model gave it
 * @param txDegrees left-right angle to the middle of the box, positive = right of center
 * @param tyDegrees up-down angle to the middle of the box, positive = above center
 */
public record CameraTarget(int classId, double txDegrees, double tyDegrees) {}

package com.pathplanner.lib.obstacle;

/**
 * A robot seen on the field.
 *
 * @param position field position, meters
 * @param color the bumper color of the robot, or UNKNOWN
 * @param timestampSec when the robot was seen, in seconds on the FPGA clock
 */
public record Obstacle(Point position, AllianceColor color, double timestampSec) {}

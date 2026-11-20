package com.pathplanner.lib.obstacle;

/**
 * A game piece seen on the field.
 *
 * @param position field position, meters
 * @param timestampSec when it was seen, in seconds on the FPGA clock
 */
public record GamePiece(Point position, double timestampSec) {}

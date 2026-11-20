package com.pathplanner.lib.obstacle;

/** A point on the field, in meters. Kept free of WPILib types so it can be tested on its own. */
public record Point(double x, double y) {}

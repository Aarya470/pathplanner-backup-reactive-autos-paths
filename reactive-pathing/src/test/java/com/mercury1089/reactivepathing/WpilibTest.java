package com.mercury1089.reactivepathing;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;

/** Starts WPILib's simulated robot once, for tests that need it. */
public abstract class WpilibTest {
  @BeforeAll
  public static void startWpilib() {
    HAL.initialize(500, 0);
  }
}

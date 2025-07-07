package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = Robot.isReal() ? Mode.REAL : simMode;
  public static final boolean tuningMode = false;
  public static final Distance botLength = Meters.of(0.87);
  public static final Distance botWidth = Meters.of(0.87);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}

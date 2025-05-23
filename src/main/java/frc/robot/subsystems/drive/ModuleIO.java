package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputes {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    public default void updateInputs(ModuleIOInputes inputs) {}

    public default void setDriveOpenLoop(double output) {}

    public default void setTurnOpenLoop(double output) {}

    public default void setDriveVelocity(double velocityRadPerSec) {}

    public default void setTurnPosition(Rotation2d rotation) {}

    public default void setDrivePID(
        double kP, double kI, double kD, double kS, double kV, double kA
    ) {}

    public default void setTurnPID(
        double kP, double kI, double kD, double kS, double kV, double kA
    ) {}
}
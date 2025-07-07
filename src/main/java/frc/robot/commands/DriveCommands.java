package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
    private static final double DEADBAND = 0.05;
    private static final double ANGLE_KP = 5.5;
    private static final double ANGLE_KI = 0.3;
    private static final double ANGLE_KD = 0.1;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    public static Command joystickDrive(
        Drive drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier) {
        return Commands.run(
        () -> {
          // Get joystick inputs
          double x = xSupplier.getAsDouble();
          double y = ySupplier.getAsDouble();

          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

          // Calculate linear velocity components
          double linearX = 0;
          double linearY = 0;

        if (linearMagnitude > 1e-6) {
            double angle = Math.atan2(y, x);
            linearX = linearMagnitude * Math.cos(angle);
            linearY = linearMagnitude * Math.sin(angle);
        }

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND) * 0.8;

          // // Square rotation value for more precise control
          // omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearX * Drive.getMaxLinearSpeedMetersPerSec(),
                  linearY * Drive.getMaxLinearSpeedMetersPerSec(),
                  omega * Drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped = true;
          // DriverStation.getAlliance().isPresent()
          //     && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.runVelocity(speeds);
        },
        drive);
    }
}

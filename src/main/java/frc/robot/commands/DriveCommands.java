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

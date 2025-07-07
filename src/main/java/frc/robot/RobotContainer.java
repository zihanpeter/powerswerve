// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;

public class RobotContainer {
  private final CommandPS5Controller driverGamepad = new CommandPS5Controller(0);

  private final Drive drive;


  public RobotContainer() {
    drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverGamepad.getLeftY(),
            () -> driverGamepad.getLeftX(),
            () -> -0.8 * driverGamepad.getRightX()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

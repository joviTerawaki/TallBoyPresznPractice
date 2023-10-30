// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.S_DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  /* * * INSTANTIATION OF OBJECTS * * */
  private XboxController xbox = new XboxController(ControllerConstants.XBOX_CONTROLLER_PORT);

  private SwerveSubsystem swerveSubs = new SwerveSubsystem();

  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(swerveSubs, () -> xbox.getLeftX(), () -> xbox.getLeftY(), () -> xbox.getRightX(), false)
      );
    // Configure the trigger bindings
    configureBindings();
  }

  //configure bindings 
  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; 
  }
}

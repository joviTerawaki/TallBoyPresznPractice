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
import frc.robot.commands.S_QuickTurnCommand;
import frc.robot.commands.S_ResetNavx;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  /* * * INSTANTIATION OF OBJECTS * * */
  private XboxController xbox = new XboxController(ControllerConstants.XBOX_CONTROLLER_PORT);

  private SwerveSubsystem swerveSubs = new SwerveSubsystem();

  private Command resetNavx = new S_ResetNavx(swerveSubs);

  public RobotContainer() {
    swerveSubs.setDefaultCommand(
      new S_DriveCommand(swerveSubs, () -> xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), true)
      );
    // Configure the trigger bindings
    configureBindings();
  }

  //configure bindings 
  private void configureBindings() {
    new JoystickButton(xbox, 1).onTrue(new InstantCommand(() -> swerveSubs.resetNavx()));
    // new JoystickButton(xbox, 3).whileTrue(new S_QuickTurnCommand(swerveSubs, () -> xbox.getLeftY(), () -> -xbox.getLeftX(), () -> -xbox.getRightX(), 180));
    // new JoystickButton(xbox, 3).(new InstantCommand(() -> swerveSubs.setDesiredAngle(swerveSubs.getRotation2d().getDegrees())));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; 
  }

  public Command getResetNavx() {
    return resetNavx; 
  }
}

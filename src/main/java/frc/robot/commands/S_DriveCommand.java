package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class S_DriveCommand extends CommandBase {
  private SwerveSubsystem swerveSubs; 

  private DoubleSupplier xSupplier, ySupplier, zSupplier; 
  private boolean fieldOriented; 

  public S_DriveCommand(SwerveSubsystem swerveSubs, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented) {
    this.swerveSubs = swerveSubs; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
    this.fieldOriented = fieldOriented; 
    addRequirements(swerveSubs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] states; 
    /* * * ALTERING VALUES * *   */
    //Joystick values -> double 
    double xSpeed = xSupplier.getAsDouble(); 
    double ySpeed = ySupplier.getAsDouble(); 
    double zSpeed = zSupplier.getAsDouble(); 

    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 
    zSpeed = deadzone(zSpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 
    zSpeed = modifyAxis(zSpeed); 

    /* * * SETTING SWERVE STATES * * */ 
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubs.getRotation2d())
      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    swerveSubs.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /* * * ADDED METHODS * * */
  public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
  }
  private static double modifyAxis(double num) {
    // Square the axis
    num = Math.copySign(num * num, num);

    return num;
  }


}

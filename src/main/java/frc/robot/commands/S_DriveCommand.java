package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public void initialize() {
    swerveSubs.resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("DONE", "not done");
    SwerveModuleState[] states; 
    /* * * ALTERING VALUES * * */
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

    // /* * * WAIALUA DESIRED ANGLE MODIFICATIONS * * */
    // //UNTESTED BE CAREFUL
    // if (zSpeed != 0) { //if rotation js is being moved 
    //   swerveSubs.desiredAngle = swerveSubs.getRotation2d().getDegrees(); //used to be .getYaw360();
    // }
   
    // swerveSubs.desiredAngle += zSpeed; 
    // swerveSubs.desiredAngle = (swerveSubs.desiredAngle + 360) % 360; //makes the desired angle positive and b/w 0 - 360
    // double angleToDesired = -wrap(swerveSubs.getRotation2d().getDegrees(), swerveSubs.desiredAngle); 
    // double rotationSpeed = angleToDesired / 90; //idk why they divide by 90??? 
    // // apply range -1 to 1
    // if (rotationSpeed > 1) rotationSpeed = 1;
    // if (rotationSpeed < -1) rotationSpeed = -1;

    /* * * SETTING SWERVE STATES * * */ 
  //   if (fieldOriented) {
  //     states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
  //       ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveSubs.getRotation2d())
  //     );
  //   } else {
  //     states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
  //       new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
  //     );
  //   }

  //   swerveSubs.setModuleStates(states);

  swerveSubs.drive(xSpeed, ySpeed, zSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubs.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("DONE", "DONE");
    return false;
  }

  /* * * ADDED METHODS * * */
  public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
  }

  private static double modifyAxis(double num) {
    // Square the axis
    num = num * num * Math.signum(num);

    return num;
  }

  //waialua wrap method from Conversions.java 
  public static double wrap(double angle1, double angle2) {
    double difference = angle1 - angle2;
    if (difference > 180) {
        difference -= 360;
        //difference = -difference;
    } else if (difference < -180) {
        difference += 360;
        //difference = -difference;
    }
    return difference;
  }


}

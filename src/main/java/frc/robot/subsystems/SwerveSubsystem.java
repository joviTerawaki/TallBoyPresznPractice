package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

  //initialize SwerveModules 
  private SwerveModule frontLeft, backLeft, frontRight, backRight; 
  private SwerveModule[] swerveModules; 

  //odometry 
  private SwerveDriveOdometry odometer; 
  private AHRS navx; 

  //desired angle 
  public double desiredAngle; 

  public SwerveSubsystem() {
    //instantiation of SwerveModules 
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, SwerveConstants.FrontLeft.constants), 
      new SwerveModule(1, SwerveConstants.BackLeft.constants), 
      new SwerveModule(2, SwerveConstants.FrontRight.constants), 
      new SwerveModule(3, SwerveConstants.BackRight.constants)
    };

    // frontLeft = new SwerveModule(
    //   SwerveConstants.FL_DRIVE_PORT,
    //   SwerveConstants.FL_ROTATION_PORT,
    //   SwerveConstants.FL_ABSOLUTE_ENCODER_PORT,
    //   SwerveConstants.FL_OFFSET,
    //   false, 
    //   true
    // ); //ba was here hahaa !!!

    // backLeft = new SwerveModule(
    //   SwerveConstants.BL_DRIVE_PORT, 
    //   SwerveConstants.BL_ROTATION_PORT, 
    //   SwerveConstants.BL_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.BL_OFFSET, 
    //   false, 
    //   true
    // );

    // frontRight = new SwerveModule(
    //   SwerveConstants.FR_DRIVE_PORT, 
    //   SwerveConstants.FR_ROTATION_PORT, 
    //   SwerveConstants.FR_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.FR_OFFSET, 
    //   false, 
    //   true
    // );

    // backRight = new SwerveModule(
    //   SwerveConstants.BR_DRIVE_PORT, 
    //   SwerveConstants.BR_ROTATION_PORT, 
    //   SwerveConstants.BR_ABSOLUTE_ENCODER_PORT, 
    //   SwerveConstants.BR_OFFSET, 
    //   false, 
    //   true
    // );

    //instantiate navx 
    navx = new AHRS();
    navx.zeroYaw();

    //instantiate odometer 
    odometer = new SwerveDriveOdometry(
      SwerveConstants.DRIVE_KINEMATICS, 
      getRotation2d(), 
      getModulePositions()
    );

    AutoBuilder.configureHolonomic(this::getPose, 
    this::resetOdometry, 
    null, null, null, null);
  }

  /* * * DRIVE * * */
  public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented) {
    SwerveModuleState[] swerveModuleStates; 

    if (fieldOriented) {
      swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationSpeed, getRotation2d())
      );
    } else {
      swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
      );
    }

    setModuleStates(swerveModuleStates);
  }

  /* * * ODOMETRY * * */

  //returns the Rotation2d object 
  //a 2d coordinate represented by a point on the unit circle (the rotation of the robot)
  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  public void resetNavx() {
    navx.reset();
  }

  public double getYaw360() {
    return Math.abs(navx.getAngle()) % 360;
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  /* * * STATES * * */

  public void setDesiredAngle(double desiredAngle) {
    this.desiredAngle = desiredAngle; 
  }

  //SET STATES 
  //gets a SwerveModuleStates array from driver control and sets each module 
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.MAX_SPEED);

    for (SwerveModule swerveMod: swerveModules) {
      swerveMod.setState(desiredStates[swerveMod.moduleNumber]);
    }
    // frontLeft.setState(desiredStates[0]);
    // backLeft.setState(desiredStates[1]);
    // frontRight.setState(desiredStates[2]);
    // backRight.setState(desiredStates[3]);
  }

  //GET STATES 
  //returns the states of the swerve modules in an array 
  //getState uses drive velocity and module rotation 
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      frontLeft.getState(), 
      backLeft.getState(), 
      frontRight.getState(), 
      backRight.getState()
    };
  }

  //GET POSITIONS
  //returns the positions of the swerve modules in an array 
  //getPosition uses drive enc and module rotation 
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeft.getPosition(), 
      backLeft.getPosition(), 
      frontRight.getPosition(), 
      backRight.getPosition()
    };
  }

  //LOCK 
  public void lock() {
    SwerveModuleState fl = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    SwerveModuleState bl = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));
    SwerveModuleState fr = new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)));
    SwerveModuleState br = new SwerveModuleState(0, new Rotation2d(Math.toRadians(-45)));

    frontLeft.setAngle(fl);
    backLeft.setAngle(bl);
    frontRight.setAngle(fr);
    backRight.setAngle(br);
  }

  //STRAIGHTEN THE WHEELS 
  public void straightenWheels() { //set all wheels to 0 degrees 
    SwerveModuleState fl = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    SwerveModuleState bl = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    SwerveModuleState fr = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));
    SwerveModuleState br = new SwerveModuleState(0, new Rotation2d(Math.toRadians(0)));

    frontLeft.setAngle(fl);
    backLeft.setAngle(bl);
    frontRight.setAngle(fr);
    backRight.setAngle(br);
  }

  //STOP 
  public void stopModules() {
    frontLeft.stop();
    backLeft.stop();
    backRight.stop();
    frontRight.stop();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometer.update(getRotation2d(), getModulePositions());
    frontLeft.print();
    backLeft.print();
    frontRight.print();
    backRight.print();
    SmartDashboard.putNumber("NAVX", navx.getYaw());
  }
}

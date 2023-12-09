package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class ControllerConstants {
    public static final int XBOX_CONTROLLER_PORT = 0;
  }

  public static class SwerveConstants {
    // public static final boolean ROTATION_ENCODER_DIRECTION = false; 

    /* * * MEASUREMENTS * * */
    //FIXME input tallboy measurements/info (got this info from alessandras old code)
    public static final double WHEEL_DIAMETER = 3 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.4318;
    public static final double WHEEL_BASE = 0.4318;
    
    public static final double DRIVE_GEAR_RATIO = 4.71 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;

    public static final double DRIVING_REDUCTION = (45.0 * 22) / (14 * 15);
    
    public static final double VOLTAGE = 8;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    // middle of roobt is (0,0)
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // front left
      // new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE/2, -WHEEL_BASE/2), 

      // // back left
      // new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE/2, -WHEEL_BASE/2),

      // // front right
      // new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE/2, WHEEL_BASE/2),

      // // back right
      // new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)
      new Translation2d(-WHEEL_BASE/2, WHEEL_BASE/2)

    );

    //OFFSETS IN RADIANS 

    /* * * FRONT LEFT * * */
    public static class FrontLeft {
      public static final int DRIVE_PORT = 1;
      public static final int ROTATION_PORT = 5;
      public static final int ABSOLUTE_ENCODER_PORT = 9;
      public static final double OFFSET = 3.481;
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK LEFT * * */
    public static class BackLeft {
      public static final int DRIVE_PORT = 2;
      public static final int ROTATION_PORT = 6;
      public static final int ABSOLUTE_ENCODER_PORT = 10;
      public static final double OFFSET = 1.219 + 0.1 + 0.023 + Math.toRadians(3.0); 
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 


      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);

    }

    /* * * FRONT RIGHT * * */
    public static class FrontRight{
    public static final int DRIVE_PORT = 4;
    public static final int ROTATION_PORT = 8;
    public static final int ABSOLUTE_ENCODER_PORT = 12;
    public static final double OFFSET = 4.536;
    public static final boolean DRIVE_INVERTED = false; 
    public static final boolean ROTATION_INVERTED = true; 

    public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }

    /* * * BACK RIGHT * * */
    public static final class BackRight {
      public static final int DRIVE_PORT = 3;
      public static final int ROTATION_PORT = 7;
      public static final int ABSOLUTE_ENCODER_PORT = 11;
      public static final double OFFSET = 1.458 - 0.05 + Math.toRadians(3);
      public static final boolean DRIVE_INVERTED = false; 
      public static final boolean ROTATION_INVERTED = true; 

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED);
    }
    
    /* * * CONVERSIONS FOR ENCODERS * * */
    //FIXME read raid zero config and figure out conversions 
    //drive position, velocity (throttle in raid zero doc)
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //(WHEEL_DIAMETER * Math.PI) / DRIVING_REDUCTION; //drive enc rotation
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 
    public static final double ROTATION_ENCODER_POSITION_CONVERSION = 6.2831854820251465; 
    public static final double ROTATION_ENCODER_VELOCITY_CONVERSION = 0.10471975803375244;
 
    //rotation position, velocity (rotor in raid zero doc)
    // public static final double ROTATION_ENCODER_POSITION_CONVERSION = 1; //STEER_GEAR_RATIO * Math.PI; //rotation enc rotation 
    // public static final double ROTATION_ENCODER_VELOCITY_CONVERSION = ROTATION_ENCODER_POSITION_CONVERSION / 60; //rotation enc speed
    // 8192 counts per motor revolution
    // 8192 * steer gear ratio  => counts per wheel revolution

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.0033;
    public static final double KI_TURNING = 0.000056;//0.00001;
    public static final double KD_TURNING = 0.00005;//0.00009;

    public static final double KP_ANGLE = 0.011; 
    public static final double KI_ANGLE = 0.000; 
    public static final double KD_ANGLE = 0.0015; 


    //FIXME speeds 
    /* * * MAX * * */
    public static final double MAX_SPEED = 3.6576;
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    
  }
}

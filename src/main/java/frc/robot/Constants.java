package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static class ControllerConstants {
    public static final int XBOX_CONTROLLER_PORT = 0;
  }

  public static class SwerveConstants {
    public static final boolean ROTATION_ENCODER_DIRECTION = false; 

    /* * * MEASUREMENTS * * */
    //FIXME input tallboy measurements/info (got this info from alessandras old code)
    public static final double WHEEL_DIAMETER = 3 * 2.5 / 100;
    public static final double TRACK_WIDTH = 0.435;
    public static final double WHEEL_BASE = 0.435;
    
    public static final double DRIVE_GEAR_RATIO = 4.71 / 1;
    public static final double ROTATION_GEAR_RATIO = 150 / 7;

    public static final double DRIVING_REDUCTION = (45.0 * 22) / (14 * 15);
    
    public static final double VOLTAGE = 8;

    /* * * SWERVE DRIVE KINEMATICS * * */
    // ORDER IS ALWAYS FL, BL, FR, BR 
    // middle of roobt is (0,0)
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      // front left
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back left
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      // front right
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      // back right
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    //FIXME ports and offsets
    /* * * FRONT LEFT * * */
    public static final int FL_DRIVE_PORT = 1;
    public static final int FL_ROTATION_PORT = 5;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 9;
    public static final double FL_OFFSET = -Math.toDegrees(1.534) + 90;

    /* * * BACK LEFT * * */
    public static final int BL_DRIVE_PORT = 2;
    public static final int BL_ROTATION_PORT = 6;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 10;
    public static final double BL_OFFSET = -Math.toDegrees(1.485) + 90 + 180;

    /* * * BACK RIGHT * * */
    public static final int BR_DRIVE_PORT = 3;
    public static final int BR_ROTATION_PORT = 7;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 11;
    public static final double BR_OFFSET = -Math.toDegrees(2.703) + 90 + 180;

    /* * * FRONT RIGHT * * */
    public static final int FR_DRIVE_PORT = 8;
    public static final int FR_ROTATION_PORT = 4;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 12;
    public static final double FR_OFFSET = -Math.toDegrees(2.637) + 90 + 180;
    
    /* * * CONVERSIONS FOR ENCODERS * * */
    //FIXME read raid zero config and figure out conversions 
    //drive position, velocity (throttle in raid zero doc)
    public static final double DRIVE_ENCODER_POSITION_CONVERSION = DRIVE_GEAR_RATIO * WHEEL_DIAMETER * Math.PI; //(WHEEL_DIAMETER * Math.PI) / DRIVING_REDUCTION; //drive enc rotation
    public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed 
 
    //rotation position, velocity (rotor in raid zero doc)
    // public static final double ROTATION_ENCODER_POSITION_CONVERSION = 1; //STEER_GEAR_RATIO * Math.PI; //rotation enc rotation 
    // public static final double ROTATION_ENCODER_VELOCITY_CONVERSION = ROTATION_ENCODER_POSITION_CONVERSION / 60; //rotation enc speed
    // 8192 counts per motor revolution
    // 8192 * steer gear ratio  => counts per wheel revolution

    /* * * PID VALUES * * */
    public static final double KP_TURNING = 0.008;//0.01; //0.1
    public static final double KI_TURNING = 0;//0.01; //0.02
    public static final double KD_TURNING = 0.0001;//0.002;

    //FIXME speeds 
    /* * * MAX * * */
    public static final double MAX_SPEED = 3.6576;
    public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    
  }
}

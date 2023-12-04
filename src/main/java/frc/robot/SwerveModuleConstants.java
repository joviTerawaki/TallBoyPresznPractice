//CLASS CONTAINING ALL SWERVE MODULE CONSTANTS 
package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int rotationMotorID;
    public final int cancoderID;
    public final double angleOffset;
    public final boolean driveInverted;
    public final boolean rotationInverted;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param rotationMotorID
     * @param canCoderID
     * @param angleOffset
     * @param driveInverted 
     * @param rotationInverted
     */
    
    public SwerveModuleConstants(int driveMotorID, int rotationMotorID, int canCoderID, double angleOffset, boolean driveInterted, boolean rotationInverted) {
        this.driveMotorID = driveMotorID;
        this.rotationMotorID = rotationMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.driveInverted = driveInterted; 
        this.rotationInverted = rotationInverted;
    }
}
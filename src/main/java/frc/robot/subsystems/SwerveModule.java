package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import frc.robot.SwerveModuleConstants;

public class SwerveModule {
    /* * * INITIALIZATION * * */
    public int moduleNumber; 
    //initialize motors 
    private CANSparkMax driveMotor; 
    private CANSparkMax rotationMotor; 

    //initialize encoders 
    private AbsoluteEncoder absoluteEncoder; 
    private RelativeEncoder driveEncoder; 
    private int absEncPort; 
    //private RelativeEncoder rotationEncoder; 

    //init PID Controller for turning 
    private PIDController rotationPID; 

    //init info 
    //private int port; 
    private double encOffset; 

    /* * * CONSTRUCTOR * * */
    /* 
     * @param drivePort port of drive motor 
     * @param rotationPort port of rotation motor 
     * @param absoluteEncoderPort port of CANCoder (absolute encoder) 
     * @param encoderOffset offset of absolute encoder 
     * @param driveInverted is the drive motor inverted? 
     * @param rotationInverted is the rotation motor inverted? 
     */
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        //port = drivePort;
        encOffset = moduleConstants.angleOffset;
        //instantiate drive motor and encoder 
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless); 
        driveEncoder = driveMotor.getEncoder();

        //instantiate rotation motor and absolute encoder 
        rotationMotor = new CANSparkMax(moduleConstants.rotationMotorID, MotorType.kBrushless);
        absoluteEncoder = rotationMotor.getAbsoluteEncoder(Type.kDutyCycle);

        //reset all motor configuration (as suggested from raid zero) (optional but safe)
        driveMotor.restoreFactoryDefaults();
        rotationMotor.restoreFactoryDefaults();

        /* * * DRIVE * * */
        //configure driving motor 
        driveMotor.setInverted(moduleConstants.driveInverted);
        driveMotor.setIdleMode(IdleMode.kBrake);

        //set conversion factor for drive enc 
        //reads velocity in meters per second instead of RPM 
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DRIVE_ENCODER_VELOCITY_CONVERSION);
        driveEncoder.setPositionConversionFactor(SwerveConstants.DRIVE_ENCODER_POSITION_CONVERSION);

        /* * * ROTATION * * */
        absEncPort = moduleConstants.cancoderID;
        //configure rotation motor 
        rotationMotor.setInverted(moduleConstants.rotationInverted);
        rotationMotor.setIdleMode(IdleMode.kBrake);

        //configure rotation absolute encoder 
        absoluteEncoder.setPositionConversionFactor(SwerveConstants.ROTATION_ENCODER_POSITION_CONVERSION); 
        absoluteEncoder.setVelocityConversionFactor(SwerveConstants.ROTATION_ENCODER_VELOCITY_CONVERSION);
        absoluteEncoder.setZeroOffset(moduleConstants.angleOffset);
        // absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180); //abs enc is now +-180 
        // absoluteEncoder.configMagnetOffset(encoderOffset); //implements encoder offset?? untested 
        // absoluteEncoder.configSensorDirection(SwerveConstants.ROTATION_ENCODER_DIRECTION); //False (default) means positive rotation occurs when magnet is spun counter-clockwise when observer is facing the LED side of CANCoder.
        // absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);

        //configure rotation PID controller 
        rotationPID = new PIDController(
            SwerveConstants.KP_TURNING, 
            SwerveConstants.KI_TURNING, 
            SwerveConstants.KD_TURNING);
        rotationPID.enableContinuousInput(-180, 180); //Continuous input considers min & max to be the same point; calculates the shortest route to the setpoint 
    } 

    /* * * GET METHODS * * */
    private double driveVelocity() {
        return driveEncoder.getVelocity();
    }

    private double drivePosition() {
        return driveEncoder.getPosition();
    }

    private double getAbsoluteEncoderDegrees() {
        return Math.toDegrees(absoluteEncoder.getPosition());
    }

    //returns a new SwerveModuleState representing the current drive velocity and rotation motor angle 
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(drivePosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDegrees()));
    }

    /* * * SET METHODS * * */

    public void setState(SwerveModuleState desiredState) {
        //optimize state so the rotation motor doesnt have to spin as much 
        //maybe should use getAbsEnc instead of getState.angle()
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate(getAbsoluteEncoderDegrees()/*getState().angle.getDegrees()*/, optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond / SwerveConstants.MAX_SPEED * SwerveConstants.VOLTAGE); 
        //driveMotor.set(optimizedState.speedMetersPerSecond); 

    }

    public void setAngle(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getState().angle);

        double rotationOutput = rotationPID.calculate( getAbsoluteEncoderDegrees()/*getState().angle.getDegrees()*/, optimizedState.angle.getDegrees());

        rotationMotor.set(rotationOutput); 
        driveMotor.set(0);
    }

    public void stop(){
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    public void print() {
        SmartDashboard.putNumber("S[" + absEncPort + "] ABS ENC RAD", Math.toRadians(getAbsoluteEncoderDegrees()));
        SmartDashboard.putNumber("S["+ absEncPort +"] DRIVE SPEED", driveVelocity());
        SmartDashboard.putNumber("S["+ absEncPort +"] ROTATION SPEED", absoluteEncoder.getVelocity());
        SmartDashboard.putString("S["+ absEncPort +"] DESIRED STATE", getState().toString());

    }
}

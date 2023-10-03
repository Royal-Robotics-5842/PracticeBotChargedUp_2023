package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    public final CANCoder CANabsoluteEncoder;

    private final PIDController turningPidController;

    //config.sensorCoefficient = 2 * Math.PI / 4096.0;

    public SwerveModule(int driveMotorId, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, int encoderId)
    {

        CANabsoluteEncoder = new CANCoder(encoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.setSmartCurrentLimit(50);
        turningMotor.setSmartCurrentLimit(50);

        driveEncoder = driveMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    
        turningPidController = new PIDController(0.007, 0,0);

        turningPidController.enableContinuousInput(-180, 180);

        CANabsoluteEncoder.configAbsoluteSensorRange(CANabsoluteEncoder.configGetAbsoluteSensorRange());

        resetEncoders();

    }

    public double getDrivePosition() 
    {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() 
    {
        return CANabsoluteEncoder.getAbsolutePosition();
        //return turnEncoder.getPosition();
    }

    public double getDriveVelocity() 
    {
        return driveEncoder.getVelocity();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        CANabsoluteEncoder.setPositionToAbsolute();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()*(Math.PI/180)));
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()*(Math.PI/180)));
        //return new SwerveModulePosition();
    }

    public void setDesiredState(SwerveModuleState state) 
    {
        state = SwerveModuleState.optimize(state, getState().angle);

        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        driveMotor.set(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
    }

    public void setSpeedTurn(double speed)
    {
        turningMotor.set(speed);
    }

    public void setSpeedDrive(double speed)
    {
        driveMotor.set(speed);
    }

    public void stop() 
    {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}

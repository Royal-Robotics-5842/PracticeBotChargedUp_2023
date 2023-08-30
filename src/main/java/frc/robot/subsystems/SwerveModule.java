package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    public final CANCoder CANabsoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed; 
    private final double absoluteEncoderOffsetAngle;

    //config.sensorCoefficient = 2 * Math.PI / 4096.0;

    public SwerveModule(int driveMotorId, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderID, double absoluteEncoderOffest, boolean absoluteEncoderReversed)
    {
        this.absoluteEncoderOffsetAngle = absoluteEncoderOffest;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        CANabsoluteEncoder = new CANCoder(absoluteEncoderID);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setSmartCurrentLimit(50);
        turningMotor.setSmartCurrentLimit(50);

        driveEncoder = driveMotor.getEncoder();
        
        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kTurnP, 0, 0);

        turningPidController.enableContinuousInput(-180, 180);

        resetEncoders();

        CANabsoluteEncoder.configAbsoluteSensorRange(CANabsoluteEncoder.configGetAbsoluteSensorRange());

    }

    public double getDrivePosition() 
    {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() 
    {
        CANabsoluteEncoder.configAbsoluteSensorRange(CANabsoluteEncoder.configGetAbsoluteSensorRange());
        return CANabsoluteEncoder.getAbsolutePosition();
    }

    public double getDriveVelocity() 
    {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad()
    {
        double angle = CANabsoluteEncoder.getAbsolutePosition();
        angle -= absoluteEncoderOffsetAngle; 
        return angle * (absoluteEncoderReversed ? -1.0: 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        CANabsoluteEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) 
    {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void setSpeed(double speed)
    {
        turningMotor.set(speed);
    }

    public void stop() 
    {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}

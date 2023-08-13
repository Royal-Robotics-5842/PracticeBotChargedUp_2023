package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class SwerveModule {
    
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogEncoder absoluteEncoder;

    private final boolean absoluteEncoderReversed; 
    private final double absoluteEncoderOffsetAngle;

    public SwerveModule(int driveMotorId, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderID, double absoluteEncoderOffest, boolean absoluteEncoderReversed)
    {
        this.absoluteEncoderOffsetAngle = absoluteEncoderOffest;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogEncoder(absoluteEncoderID);

        driveMotor = new CANSparkMax(driveMotorId,MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();
        
        //driveEncoder.setPositionConversionFactor();
    }
}

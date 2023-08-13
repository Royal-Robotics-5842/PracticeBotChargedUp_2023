// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  private final CANSparkMax mLeftTop = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax mLeftBottom = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax mRightTop = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax mRightBottom = new CANSparkMax(1, MotorType.kBrushless);

  private RelativeEncoder mLeftEncoderTop = mLeftTop.getEncoder();
  private RelativeEncoder mLeftEncoderBottom = mLeftBottom.getEncoder();
  private RelativeEncoder mRightEncoderTop = mRightTop.getEncoder();
  private RelativeEncoder mRightEncoderBottom = mRightBottom.getEncoder();


  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  public drivetrain() 
  {
    mLeftTop.restoreFactoryDefaults();
    mLeftBottom.restoreFactoryDefaults();
    mRightTop.restoreFactoryDefaults();
    mRightBottom.restoreFactoryDefaults();

    mLeftTop.setSmartCurrentLimit(15);
    mLeftBottom.setSmartCurrentLimit(15);
    mRightTop.setSmartCurrentLimit(15);
    mRightBottom.setSmartCurrentLimit(15);  

    mLeftTop.setIdleMode(IdleMode.kBrake);
    mLeftBottom.setIdleMode(IdleMode.kBrake);
    mRightTop.setIdleMode(IdleMode.kBrake);
    mRightBottom.setIdleMode(IdleMode.kBrake);

    mLeftEncoderTop.setPosition(0);
    mLeftEncoderBottom.setPosition(0);
    mRightEncoderTop.setPosition(0);
    mRightEncoderBottom.setPosition(0);

    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

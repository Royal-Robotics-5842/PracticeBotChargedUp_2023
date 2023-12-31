// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsytem extends SubsystemBase {
  public final CANSparkMax rightMotor = new CANSparkMax(39, MotorType.kBrushless);
  public final CANSparkMax leftMotor = new CANSparkMax(38, MotorType.kBrushless);

  public IntakeSubsytem() 
  {
    
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();

    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setSpeed(double speed)
  {
    rightMotor.set(speed);
    leftMotor.set(speed);
  }

  public void stopMotors()
  {
    rightMotor.set(0);
    leftMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("XController" , rightMotor.getAppliedOutput());
  }
}

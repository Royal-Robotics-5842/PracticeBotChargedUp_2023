// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class Speed extends CommandBase {
  /** Creates a new Speed. */

    private final SwerveSubsystem swerve;

  public Speed(SwerveSubsystem swerve) 
  {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      swerve.backLeft.stop();
    }
    /*
    swerve.backLeft.setSpeed(0.1);
    swerve.backRight.setSpeed(0.1);
    swerve.frontLeft.setSpeed(0.1);
    swerve.frontRight.setSpeed(0.1);
  */
    
   //System.out.println(swerve.frontLeft.CANabsoluteEncoder.setPosition(0));

    System.out.println("Turning Encoder Value " + swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() > 0
        && swerve.backRight.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.backRight.CANabsoluteEncoder.getAbsolutePosition() > 0
        && swerve.frontLeft.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.frontLeft.CANabsoluteEncoder.getAbsolutePosition() > 0
        && swerve.frontRight.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.frontRight.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      return true;
    }
    return false;
  }
}

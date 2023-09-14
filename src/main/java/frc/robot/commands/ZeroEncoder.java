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

public class ZeroEncoder extends CommandBase {
  /** Creates a new Speed. */
  private ChassisSpeeds chassisSpeeds;
    private final SwerveSubsystem swerve;

  public ZeroEncoder(SwerveSubsystem swerve) 
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
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerve.setModuleStates(moduleStates);
    //System.out.println("turningSpeed " + moduleStates);
    /*
    if (swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() < 3 && swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      swerve.backLeft.stop();
    }
    else
    {
      swerve.backLeft.setSpeedTurn(0.05);
    }

    if (swerve.backRight.CANabsoluteEncoder.getAbsolutePosition() < 3 && swerve.backRight.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      swerve.backRight.stop();
    }
    else
    {
      swerve.backRight.setSpeedTurn(0.05);
    }
    if (swerve.frontLeft.getTurningPositionFL() < 3 && swerve.frontLeft.getTurningPositionFL() > 0)
    {
      swerve.frontLeft.stop();
    }
    else
    {
      swerve.frontLeft.setSpeedTurn(0.05);
    }
    

    if (swerve.frontRight.CANabsoluteEncoder.getAbsolutePosition() < 3 && swerve.frontRight.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      swerve.frontRight.stop();
    }
    else
    {
      swerve.frontRight.setSpeedTurn(0.05);
    }
    
    */
   //System.out.println(swerve.frontLeft.CANabsoluteEncoder.setPosition(0));

    System.out.println("Turning Encoder Value " + swerve.frontLeft.CANabsoluteEncoder.getAbsolutePosition());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if (swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() < 5 && swerve.backLeft.CANabsoluteEncoder.getAbsolutePosition() > 0)
    {
      System.out.print("done");
      return true;
    }
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveJoystick extends CommandBase {
  
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;

  private ChassisSpeeds chassisSpeeds;

  public SwerveDriveJoystick(SwerveSubsystem swerveSubsystem,
  Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction) {
              this.swerveSubsystem = swerveSubsystem;
              this.xSpdFunction = xSpdFunction;
              this.ySpdFunction = ySpdFunction;
              this.turningSpdFunction = turningSpdFunction;

    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(swerveSubsystem);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // 1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get()*0.75;

    xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;


    //System.out.println("xSpeed " + xSpeed);
    //System.out.println("ySpeed " + ySpeed);

    /*
    if(xSpeed == 0)
    {
      swerveSubsystem.frontLeft.stop();
      swerveSubsystem.frontRight.stop();
      swerveSubsystem.backLeft.stop();
      swerveSubsystem.backRight.stop();
    }

    if(turningSpeed == 0)
    {
      swerveSubsystem.frontLeft.stop();
      swerveSubsystem.frontRight.stop();
      swerveSubsystem.backLeft.stop();
      swerveSubsystem.backRight.stop();
    } 

    if(xSpeed > 0 || xSpeed < 0)
    {
      swerveSubsystem.frontLeft.setSpeedDrive(xSpeed);
      swerveSubsystem.frontRight.setSpeedDrive(xSpeed);
      swerveSubsystem.backLeft.setSpeedDrive(xSpeed);
      swerveSubsystem.backRight.setSpeedDrive(xSpeed);
      
    }
    if (turningSpeed > 0 || turningSpeed < 0)
    {
      swerveSubsystem.frontLeft.setSpeedTurn(turningSpeed);
      swerveSubsystem.frontRight.setSpeedTurn(turningSpeed);
      swerveSubsystem.backLeft.setSpeedTurn(turningSpeed);
      swerveSubsystem.backRight.setSpeedTurn(turningSpeed);
    }
    */
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
  

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
    //System.out.println("turningSpeed " + moduleStates);
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

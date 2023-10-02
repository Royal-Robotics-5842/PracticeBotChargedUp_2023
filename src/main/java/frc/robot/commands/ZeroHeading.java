// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ZeroHeading extends CommandBase {
  private final SwerveSubsystem swerve;
  public ZeroHeading(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    swerve.zeroHeading();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (swerve.gyro.getYaw() == 0)
    {
      System.out.print("Zero Heading Done!");
      return true;
    }

    return false;
  }
}

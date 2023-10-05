// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EverythingSwerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SetSwerveSpeed extends CommandBase {
  private final SwerveSubsystem swerve;
  public SetSwerveSpeed(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    swerve.frontLeft.setSpeedDrive(0.3);
    swerve.frontRight.setSpeedDrive(0.3);
    swerve.backLeft.setSpeedDrive(0.3);
    swerve.backRight.setSpeedDrive(0.3);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

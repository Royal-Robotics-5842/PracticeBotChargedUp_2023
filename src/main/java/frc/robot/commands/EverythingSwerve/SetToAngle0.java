// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EverythingSwerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SetToAngle0 extends CommandBase {
  private final SwerveSubsystem swerve;
  public SetToAngle0(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    swerve.frontLeft.setToAngle(0);
    swerve.frontRight.setToAngle(0);
    swerve.backLeft.setToAngle(0);
    swerve.backRight.setToAngle(0);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (swerve.frontLeft.getTurningPosition() <= 1 && swerve.frontLeft.getTurningPosition() <= -0.5)
    {
      System.out.print("Swerve to 0");
      return true;
    }

    return false;
  }
}

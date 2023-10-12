// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsytem;


public class IntakeSetSpeed extends CommandBase {
 
  public final IntakeSubsytem intake;
  public double intakeSpeed;
  private double timeBeforeStop;


  public IntakeSetSpeed(IntakeSubsytem intake, double speed) {
    this.intake = intake;
    intakeSpeed = speed;
    addRequirements(intake);
    // Use addRequirements() herex to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    //timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    intake.setSpeed(intakeSpeed);
    SmartDashboard.putBoolean("Intake With Triggers", (intakeSpeed) <= 0.1 && Math.abs(intakeSpeed) <= 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (Timer.getMatchTime() >= timeBeforeStop)
    //{
    //  return true;
    //}
    return false;
  }
}

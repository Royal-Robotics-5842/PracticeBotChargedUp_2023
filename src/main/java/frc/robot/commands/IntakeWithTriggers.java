// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsytem;

public class IntakeWithTriggers extends CommandBase {
  /** Creates a new IntakeWithTriggers. */
  public final IntakeSubsytem intake;
  private final double rightTrigger, leftTrigger;
  
  public IntakeWithTriggers(IntakeSubsytem intake, double leftTrigger, double rightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;

    this.rightTrigger = rightTrigger;
    this.leftTrigger = leftTrigger;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double leftSpeed = leftTrigger;
    double rightSpeed = rightTrigger;

    if (leftSpeed > 0)
      intake.setSpeed(leftSpeed);
    if (rightSpeed > 0)
      intake.setSpeed(rightSpeed);
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
    return false;
  }
}

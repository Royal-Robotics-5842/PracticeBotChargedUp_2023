// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EverythingSwerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsytem;

public class IntakeWithTriggers extends CommandBase {
  /** Creates a new IntakeWithTriggers. */
  public final IntakeSubsytem intake;
  private final double leftTrigger;
  private final double rightTrigger;
  public IntakeWithTriggers(IntakeSubsytem intake, double left, double right) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    leftTrigger = left;
    rightTrigger = right;
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //System.out.println(RobotContainer.m_driverController.getLeftTriggerAxis() + " left : right " + RobotContainer.m_driverController.getRightTriggerAxis());
    if (RobotContainer.m_driverController.getLeftTriggerAxis() >= 0)
    {
      intake.setSpeed(RobotContainer.m_driverController.getLeftTriggerAxis() * -1);

    }
    if (RobotContainer.m_driverController.getRightTriggerAxis() > 0) // added a =
    {
      intake.setSpeed(RobotContainer.m_driverController.getRightTriggerAxis() * 0.2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

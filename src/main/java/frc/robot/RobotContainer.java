// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.EverythingSwerve.IntakeWithTriggers;
//import frc.robot.commands.AutoCommands.IntakeSetSpeed;
import frc.robot.commands.EverythingSwerve.SetToAngle0;
import frc.robot.commands.EverythingSwerve.SetToX;
import frc.robot.commands.EverythingSwerve.StopSwerveModule;
import frc.robot.commands.EverythingSwerve.SwerveDriveJoystick;
import frc.robot.commands.EverythingSwerve.ZeroHeading;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems are defined here...
  private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsytem intake = new IntakeSubsytem();


  //THe robot's commands are defined here...
  private final ZeroHeading zeroHeading = new ZeroHeading(swerveSubsystem);
  private final SetToAngle0 setTo0 = new SetToAngle0(swerveSubsystem);
  private final SetToX setToX = new SetToX(swerveSubsystem); //ALWAYS RUN WITH TIMEOUT


  //All our controller stuff!
  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  public final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public final static double lefTrigger = m_driverController.getLeftTriggerAxis();
  public final static double rightTrigger = m_driverController.getRightTriggerAxis();


  static SendableChooser<Command> auto_chooser = new SendableChooser<>(); //Initializing the autonomous chooser on smartdashboard

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    swerveSubsystem.setDefaultCommand(new SwerveDriveJoystick(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () ->  driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !m_driverController.y().getAsBoolean()));//driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      intake.setDefaultCommand(new IntakeWithTriggers(intake, lefTrigger, rightTrigger));


      SmartDashboard.putBoolean("Field Centric", !m_driverController.y().getAsBoolean());
      SmartDashboard.putNumber("Robot Pitch", swerveSubsystem.gyro.getPitch());


      auto_chooser.setDefaultOption("No Intake - Straight Side", Autos.Straight(swerveSubsystem));
      auto_chooser.addOption("Side_DriveStraight", Autos.Side_DriveStraight(swerveSubsystem, intake));
      auto_chooser.addOption("Middle_AutoBalance", Autos. Middle_AutoBalance(swerveSubsystem, intake));
      auto_chooser.addOption("TESTTT", loadPathplannerTrajectoryToHolonomicCommand("C:\\Users\\royalrobotics\\Desktop\\PracticeBotChargedUp_2023\\src\\main\\deploy\\deploy\\pathplanner\\generatedJSON\\New Path.wpilib.json"
      , true));
      //auto_chooser.addOption("back -- THIS IS FOR TESTING DO NOT RUN THIS AT ALL", Autos.StraightBack(swerveSubsystem));
      SmartDashboard.putData(auto_chooser);



    configureBindings();
  }

  public static Command loadPathplannerTrajectoryToHolonomicCommand(String filename, boolean resetOdomtry) {
    Trajectory trajectory;
    System.out.println("HIIIIIIIII");
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-180, 180);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException exception) {
      DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
      System.out.println("Unable to read from file " + filename);
      return new InstantCommand();
    }
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                  trajectory,
                  swerveSubsystem::getPose,
                  DriveConstants.kDriveKinematics,
                  new PIDController(AutoConstants.kPXController, 0, 0),
                  new PIDController(AutoConstants.kPYController, 0, 0),
                  thetaController,
                  swerveSubsystem::setModuleStates,
                  swerveSubsystem);

                if (resetOdomtry) {
                  return new SequentialCommandGroup(
                       new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())), swerveControllerCommand);
                 } else {
                   return swerveControllerCommand;
                 }
}


  private void configureBindings() {
    m_driverController.a().onTrue(zeroHeading); 
    m_driverController.x().onTrue(setToX.withTimeout(0.5));
    m_driverController.b().onTrue(setTo0.withTimeout(0.5));
  }
    
    
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.EverythingSwerve.SetToAngle0;
import frc.robot.commands.EverythingSwerve.SetToX;
import frc.robot.commands.EverythingSwerve.StopSwerveModule;
import frc.robot.commands.EverythingSwerve.SwerveDriveJoystick;
import frc.robot.commands.EverythingSwerve.ZeroHeading;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsytem intake = new IntakeSubsytem();


  //THe robot's commands are defined here...
  private final ZeroHeading zeroHeading = new ZeroHeading(swerveSubsystem);
  private final StopSwerveModule stopSwerve = new StopSwerveModule(swerveSubsystem);
  private final SetToAngle0 setTo0 = new SetToAngle0(swerveSubsystem);
  private final SetToX setToX = new SetToX(swerveSubsystem);
  //private final IntakeSetSpeed iSetSpeed = new IntakeSetSpeed(intake, 0);


  //All our controller stuff!
  public final static Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final static CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
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

      //intake.setDefaultCommand(new IntakeWithTriggers(intake, lefTrigger, rightTrigger));


      SmartDashboard.putBoolean("Field Centric", !m_driverController.y().getAsBoolean());
      SmartDashboard.putNumber("Robot Pitch", swerveSubsystem.gyro.getPitch());


      auto_chooser.setDefaultOption("No Intake - Straight Side", Autos.Straight(swerveSubsystem));
      auto_chooser.addOption("Side_DriveStraight", Autos.Side_DriveStraight(swerveSubsystem, intake));
      auto_chooser.addOption("Middle_AutoBalance", Autos. Middle_AutoBalance(swerveSubsystem, intake));
      //auto_chooser.addOption("back -- THIS IS FOR TESTING DO NOT RUN THIS AT ALL", Autos.StraightBack(swerveSubsystem));
      SmartDashboard.putData(auto_chooser);



    configureBindings();
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.a().onTrue(zeroHeading); 
    m_driverController.x().onTrue(setToX);
    m_driverController.b().onTrue(setTo0);
  }
    
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the scommand to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}


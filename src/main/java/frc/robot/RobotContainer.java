// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeWithTriggers;
import frc.robot.commands.SwerveDriveJoystick;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.setTo45;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsytem;
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
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
<<<<<<< HEAD
  private final IntakeSubsytem IntakeSubsytem = new IntakeSubsytem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
=======
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
>>>>>>> fecd7703883aa159f8fe933d030c0c2d5a5c98dc
  private final ZeroHeading zeroHeading = new ZeroHeading(swerveSubsystem);
  private final setTo45 setTo45 = new setTo45(swerveSubsystem);
  private final IntakeSubsytem intake = new IntakeSubsytem();
  //private final IntakeWithTriggers intakeWithTriggers = new IntakeWithTriggers(IntakeSubsytem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final Trigger yButton = m_driverController.y();


  SendableChooser<Command> auto_chooser = new SendableChooser<>();
  //rivate final Field2d m_field = new Field2d();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveDriveJoystick(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !yButton.getAsBoolean()));//driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));



      SmartDashboard.putBoolean("Field Centric", yButton.getAsBoolean());
    
      
      Autos.Straight(swerveSubsystem).setName("Straight");
      Autos.RunIntakeandTraj(swerveSubsystem, intake).setName("Intake");
      auto_chooser.setDefaultOption("Auto 1", Autos.Straight(swerveSubsystem));
      auto_chooser.addOption("Auto 2", Autos.RunIntakeandTraj(swerveSubsystem, intake));
      SmartDashboard.putData(auto_chooser);
<<<<<<< HEAD
      () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

      IntakeSubsytem.setDefaultCommand(new IntakeWithTriggers(IntakeSubsytem, 
                                      m_driverController.getLeftTriggerAxis(),
                                      m_driverController.getRightTriggerAxis()));
=======

>>>>>>> fecd7703883aa159f8fe933d030c0c2d5a5c98dc
    // Configure the tri gger bindings
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
    m_driverController.x().onTrue(setTo45);
  
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


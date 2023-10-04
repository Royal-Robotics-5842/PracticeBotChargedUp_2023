// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoCommands.AutoBalance;
//import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.IntakeSetSpeed;
import frc.robot.subsystems.IntakeSubsytem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {


  private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //private final static IntakeSubsytem intake = new IntakeSubsytem();

/*
  public static CommandBase Straight(SwerveSubsystem swerveSubsystem) {
                // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                  AutoConstants.kMaxSpeedMetersPerSecond,
                  AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                          .setKinematics(DriveConstants.kDriveKinematics);

                ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
                interiorWaypoints.add(new Translation2d(1, 0));
                interiorWaypoints.add(new Translation2d(2, 0));
                interiorWaypoints.add(new Translation2d(3,0));


                // 2. Generate trajectory
                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                  new Pose2d(0, 0, new Rotation2d(0)),
                  interiorWaypoints,
                  new Pose2d(4,0, Rotation2d.fromDegrees(90)),
                  trajectoryConfig);


                // 3. Define PID controllers for tracking trajectory
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-180, 180);

                // 4. Construct command to follow trajectory
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                  trajectory,
                  swerveSubsystem::getPose,
                  DriveConstants.kDriveKinematics,
                  xController,
                  yController,
                  thetaController,
                  swerveSubsystem::setModuleStates,
                  swerveSubsystem);

                  SmartDashboard.putData("XController" , xController);
                  SmartDashboard.putData("YController" , yController);
                  SmartDashboard.putData("ThetaController" , thetaController);
                  
                  return Commands.sequence(
                            new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                            swerveControllerCommand,
                            new InstantCommand(() -> swerveSubsystem.stopModules()));
      }

      public static CommandBase StraightBack(SwerveSubsystem swerveSubsystem) {
                      // 1. Create trajectory settings
                      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(DriveConstants.kDriveKinematics);

                      ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
                      interiorWaypoints.add(new Translation2d(3, 0));
                      interiorWaypoints.add(new Translation2d(2, 0));
                      interiorWaypoints.add(new Translation2d(1,0));


                      // 2. Generate trajectory
                      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(4, 0, swerveSubsystem.getRotation2d()),
                        interiorWaypoints,
                        new Pose2d(0,0, Rotation2d.fromDegrees(0)),
                        trajectoryConfig);


                      // 3. Define PID controllers for tracking trajectory
                      PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                      PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                      ProfiledPIDController thetaController = new ProfiledPIDController(
                              AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                      thetaController.enableContinuousInput(-180, 180);

                      // 4. Construct command to follow trajectory
                      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                        trajectory,
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem);

                        SmartDashboard.putData("XController" , xController);
                        SmartDashboard.putData("YController" , yController);
                        SmartDashboard.putData("ThetaController" , thetaController);
                        
                        return Commands.sequence(
                                  new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                                  swerveControllerCommand,
                                  new InstantCommand(() -> swerveSubsystem.stopModules()));
      }
      */
    
      /*
      public static CommandBase Side_DriveStraight(SwerveSubsystem swerveSubsystem, IntakeSubsytem intake)
      {
        return new RunCommand(() -> swerveSubsystem.autoBalance(), swerveSubsystem);//   new AutoBalance(swerveSubsystem)); //intake.setSpeed(1), intake), new WaitCommand(5), new IntakeSetSpeed(intake, 0));// Straight(swerveSubsystem));
      } 
      
      
/*  
      public static CommandBase Middle_AutoBalance(SwerveSubsystem swerveSubsystem, IntakeSubsytem intake)
      {
        return Commands.sequence(
                  new IntakeSetSpeed(intake, 1 ), new WaitCommand(5), new IntakeSetSpeed(intake, 0 ),
                  Straight(swerveSubsystem).until(() -> swerveSubsystem.gyro.getPitch() > 6).andThen(new AutoBalance(swerveSubsystem)));
      } 
      */
      
      
      
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");

  }


}

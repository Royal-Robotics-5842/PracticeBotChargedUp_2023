// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PoseEstimatorSubsystem extends SubsystemBase {
  /** Creates a new LimeLight. */
 
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight");
  DoubleArraySubscriber a = table.getDoubleArrayTopic("area").subscribe(new double[] {});

  public PoseEstimatorSubsystem() 
  {}

  public void getPose4()
  {
    double[] hi = a.get();
    Translation3d translation = new Translation3d(hi[0],hi[1],hi[2]);
    Rotation3d rotation = new Rotation3d(hi[3],hi[4],hi[5]);
    Pose3d limelightpose = new Pose3d(translation, rotation);

    double x = limelightpose.getX();
    double y = limelightpose.getY();

    System.out.print("(" + x + ", " + y + ")"); /*prints out coordinates */
    System.out.println("HI");
  
  }
  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
  }
}

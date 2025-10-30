// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotPositioningSystem extends SubsystemBase {
  VisionSystem vision;
  CommandSwerveDrivetrain drivetrain;
  Pose2d currentPose;
  
  /** Creates a new RobotPositioningSystem. */
  public RobotPositioningSystem(CommandSwerveDrivetrain drivetrain, VisionSystem vision) {
    this.drivetrain = drivetrain;
    this.vision = vision;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentPose = drivetrain.getState().Pose;
    SmartDashboard.putString("Odometry Pose", currentPose.toString());
    if(vision.hasValidTarget()){
      currentPose = vision.getRobotPose().get().toPose2d();
      drivetrain.addVisionMeasurement(currentPose, Utils.getCurrentTimeSeconds());
      SmartDashboard.putString("Vision Pose", currentPose.toString());
  }
  SmartDashboard.putString("Robot Pose", currentPose.toString());

}
}
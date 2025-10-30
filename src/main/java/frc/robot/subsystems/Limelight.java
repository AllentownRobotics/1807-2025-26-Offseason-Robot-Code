// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers;

public class Limelight extends SubsystemBase implements Camera {
  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  
  @Override
  public Pose3d getTargetPose() {
    return LimelightHelpers.getTargetPose3d_RobotSpace("limelight");
  }

  @Override
  public void setCameraPose(Pose3d cameraPose) {
    LimelightHelpers.setCameraPose_RobotSpace("limelight", cameraPose.getZ(), cameraPose.getX(), cameraPose.getY(),
        cameraPose.getRotation().getZ(), cameraPose.getRotation().getX(), cameraPose.getRotation().getY());
  }

  @Override
  public int getTargetID() {
    return LimelightHelpers.getLimelightNTTable("limelight").getEntry("tid").getNumber(-1).intValue();
  }

  @Override
  public boolean hasTarget() {
    return LimelightHelpers.getLimelightNTTable("limelight").getEntry("tid").getNumber(-1).intValue() >= 0;
  }

  
}

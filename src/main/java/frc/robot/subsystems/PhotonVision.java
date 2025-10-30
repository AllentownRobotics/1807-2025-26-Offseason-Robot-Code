// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase implements Camera {
  private PhotonCamera camera;
  /** Creates a new PhotonVision. */
  public PhotonVision(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public Pose3d getTargetPose() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTargetPose'");
  }

  @Override
  public void setCameraPose(Pose3d cameraPose) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setCameraPose'");
  }

  @Override
  public int getTargetID() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getTargetID'");
  }

  @Override
  public boolean hasTarget() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'hasTarget'");
  }

  
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;

public class VisionSystem extends SubsystemBase {
  Camera[] cameras;
  /**
   * Map of target IDs to their poses on the field in inches. In initializer, converts to meters.
   */
  HashMap<Integer, Pose3d> targetPoses = VisionConstants.TargetPoses.targetPosesMap;
  /**
   * List of target poses and ids detected by the cameras.
   */
  HashMap<Integer, Pose3d> detectedTargetPoses = new HashMap<Integer, Pose3d>();
  /**
   * robot position on the field, using WPIlib field coordinate system.
   */
  Pose3d robotPose = new Pose3d();

  /** Creates a new VisionSystem. */
  public VisionSystem() {
    targetPoses.replaceAll((k,v) -> new Pose3d(v.getTranslation().div(39.3700787), v.getRotation()));
    cameras = new Camera[] {
      VisionConstants.Cameras.Camera1,
      VisionConstants.Cameras.Camera2,
      VisionConstants.Cameras.Camera3
    };
      
    }

  /**
   * Gets the target poses detected by the cameras and their IDs.
   * @return
   */
  private HashMap<Integer, Pose3d> getTargetPoses(){
    detectedTargetPoses.clear();
    for (Camera camera : cameras) {
      if (camera.hasTarget()) {
        int targetID = camera.getTargetID();
        if (targetPoses.containsKey(targetID)) {
          detectedTargetPoses.put(targetID, camera.getTargetPose());
        }
      }
    }
    return detectedTargetPoses;
  }

  public Optional<Pose3d> getRobotPose() {
    getTargetPoses();
    ArrayList<Pose3d> calculatedRobotPoses = new ArrayList<Pose3d>();
    for (Integer targetID : detectedTargetPoses.keySet()) {
      Pose3d cameraToTarget = detectedTargetPoses.get(targetID);
      Pose3d fieldTargetPose = targetPoses.get(targetID);
      Pose3d cameraPose = fieldTargetPose.transformBy(new Transform3d(cameraToTarget.div(-1).getTranslation(), cameraToTarget.div(-1).getRotation()));
      // For now, assume the camera is at the robot origin. In the future, we can add the camera pose relative to the robot.
      calculatedRobotPoses.add(cameraPose);
    }
    
      double x = 0;
      double y = 0;
      double z = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;
      for (Pose3d pose : calculatedRobotPoses) {
        x += pose.getX();
        y += pose.getY();
        z += pose.getZ();
        roll += pose.getRotation().getX();
        pitch += pose.getRotation().getY();
        yaw += pose.getRotation().getZ();
      }
      x /= calculatedRobotPoses.size();
      y /= calculatedRobotPoses.size();
      z /= calculatedRobotPoses.size();
      roll /= calculatedRobotPoses.size();
      pitch /= calculatedRobotPoses.size();
      yaw /= calculatedRobotPoses.size();
      robotPose = new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
      return robotPose == null ? Optional.empty() : Optional.of(robotPose);
    
    
  }

  /**
   * returns true if at least one camera has a target in view.
   */
  public boolean hasValidTarget() {
    for (Camera camera : cameras) {
      if (camera.hasTarget()) {
        return true;
      }
    }
    return false;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

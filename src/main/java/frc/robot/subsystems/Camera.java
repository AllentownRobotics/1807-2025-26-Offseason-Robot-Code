package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface Camera {

    /**Returns the position of the target in 3D space relative to the
   * robot in meters. Camera position must first be set to get the
   * target position in the robots coordinate system. Otherwise the
   * target position will be relative to the camera.
   * @return Pose3d of the target in meters
   */
    public Pose3d getTargetPose();

    /**
     * Sets the position of the camera in 3D space relative to the robot in meters.
     * If the camera position is not set, all values will be relative to the camera.
     * @param cameraPose
     */
    public void setCameraPose(Pose3d cameraPose);

    /**
     * Returns the ID of the target being tracked. If no target is being tracked, returns -1.
     * @return target ID or -1 if no target is being tracked
     */
    public int getTargetID();

    /**
     * Returns true if the camera has a target in view, false otherwise.
     * @return true if the camera has a target in view, false otherwise
     */
    public boolean hasTarget();
}

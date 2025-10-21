package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonVision;

public final class VisionConstants {
    
    public final class TargetPoses {
        public static final HashMap<Integer, Pose3d> targetPosesMap = new HashMap<Integer, Pose3d>() {{
            put(1, new Pose3d(657.37, 25.8, 58.5, new Rotation3d(126, 0, 0)));
            put(2, new Pose3d(657.37, 241.2, 58.5, new Rotation3d(234, 0, 0)));
            put(3, new Pose3d(455.15, 317.15, 51.25, new Rotation3d(270, 0, 0)));
            put(4, new Pose3d(365.2, 241.64, 73.54, new Rotation3d(0, 0, 30)));
            put(5, new Pose3d(365.2, 75.39, 73.54, new Rotation3d(0, 0, 30)));
            put(6, new Pose3d(530.49, 130.17, 12.13, new Rotation3d(300, 0, 0)));
            put(7, new Pose3d(546.87, 130.17, 12.13, new Rotation3d(0, 0, 0)));
            put(8, new Pose3d(530.49, 186.83, 12.13, new Rotation3d(60, 0, 0)));
            put(9, new Pose3d(497.77, 186.83, 12.13, new Rotation3d(120, 0, 0)));
            put(10, new Pose3d(481.39, 158.5, 12.13, new Rotation3d(180, 0, 0)));
            put(11, new Pose3d(497.77, 130.17, 12.13, new Rotation3d(240, 0, 0)));
            put(12, new Pose3d(33.51, 25.80, 58.50, new Rotation3d(54, 0, 0)));
            put(13, new Pose3d(33.51, 291.20, 58.50, new Rotation3d(306, 0, 0)));
            put(14, new Pose3d(325.68, 241.64, 58.50, new Rotation3d(180, 0, 30)));
            put(15, new Pose3d(325.68, 75.39, 73.54, new Rotation3d(180, 0, 30)));
            put(16, new Pose3d(235.73, -0.15, 73.54, new Rotation3d(90, 0, 0)));
            put(17, new Pose3d(160.39, 130.17, 51.25, new Rotation3d(240, 0, 0)));
            put(18, new Pose3d(144.00, 158.50, 12.13, new Rotation3d(180, 0, 0)));
            put(19, new Pose3d(160.39, 186.83, 12.13, new Rotation3d(120, 0, 0)));
            put(20, new Pose3d(193.10, 186.83, 12.13, new Rotation3d(60, 0, 0)));
            put(21, new Pose3d(209.49, 158.50, 12.13, new Rotation3d(0, 0, 0)));
            put(22, new Pose3d(193.10, 130.17, 12.13, new Rotation3d(300, 0, 0)));
            
        }}; // in inches
        
        
    }
    public final class Cameras{
        public static final Camera Camera1 = new Limelight();
        static{
            Camera1.setCameraPose(new Pose3d( // inches
                15.5, // forward
                0,    // left
                24,   // up
                new Rotation3d(0, 0, 0) // facing forward
            ));
        }

        public static final Camera Camera2 = new Limelight();
        static{
            Camera2.setCameraPose(new Pose3d( // inches
                15.5, // forward
                0,    // left
                24,   // up
                new Rotation3d(0, 0, 0) // facing forward
            ));
        }
        
        public static final Camera Camera3 = new Limelight();
        static{
            Camera3.setCameraPose(new Pose3d( // inches
                15.5, // forward
                0,    // left
                24,   // up
                new Rotation3d(0, 0, 0) // facing forward
            ));
        }
    }
}

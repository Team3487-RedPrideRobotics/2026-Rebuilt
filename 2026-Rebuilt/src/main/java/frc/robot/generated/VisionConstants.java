package frc.robot.generated;

import java.util.Map;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;


public class VisionConstants {
    //limelight-chassis/limelight-front
    public static final Transform3d llCameraRightPose = new Transform3d(Units.inchesToMeters(-3.818)
                                                        ,  Units.inchesToMeters(-11.457) 
                                                        ,  Units.inchesToMeters(9)
                                                        ,  new Rotation3d(0, -Math.PI/8, -Math.PI*9/12));
    public static final String llProcessorRightID =  "limelight-chassis";
    //limelight-left
    public static final Transform3d llCameraLeftPose = new Transform3d(Units.inchesToMeters(-9.48)
                                                        ,  Units.inchesToMeters(12.714) 
                                                        ,  Units.inchesToMeters(9)
                                                        ,  new Rotation3d(0,0,Math.PI/2));
    public static final String llProcessorLeftID =  "limelight-shooter";

    public static final AprilTagFieldLayout AprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
    public static final Matrix<N3,N1> SingleTagStdDevs = VecBuilder.fill(3, 3, 3);
    public static final Matrix<N3,N1> MultiTagStdDevs = VecBuilder.fill(1, 1, 1);

    public static final SimCameraProperties llCameraSimProperties = new SimCameraProperties()
                                            .setCalibration(1280, 800,Rotation2d.fromDegrees(99.4))
                                            .setCalibError(0.35, 0.01)
                                            .setFPS(20)
                                            .setAvgLatencyMs(50)
                                            .setLatencyStdDevMs(15);

    //Global pose of the hubs
    public static final Pose2d BlueHubPose2d = new Pose2d(4.6228,4.034536,new Rotation2d(0));
    public static final Pose2d RedHubPose2d = new Pose2d(11.9126,4.034536,new Rotation2d(0));

    //Goal pose for passing to alliance zone
    public static final Rotation2d BlueAlliancePassRotation2d = Rotation2d.k180deg;
    public static final Rotation2d RedAlliancePassRotation2d = Rotation2d.kZero;


    //Interpolator values 
    //WARNING: All current values do not actually represent the current robot's data!

    //Hood interpolator; distance to goal in (meters), pitch out (degrees)
    public static final InterpolatingDoubleTreeMap TurretHoodInterpolatorDEG = InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.13, 30.0)
         ,Map.entry(4.57,36.0)
         ,Map.entry(4.87,32.0)
        );


    //Flywheel Interpolator; distance to goal(meters), Speed out (RPM)
    public static final InterpolatingDoubleTreeMap TurretFlywheelInterpolatorRPM = InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.13, 3100.0)
         ,Map.entry(2.28,3100.0)
         ,Map.entry(2.43,3250.0)
         ,Map.entry(2.49,3200.0)
         ,Map.entry(2.74,3430.0)
         ,Map.entry(3.0,3440.0)
         ,Map.entry(3.2,3550.0)
         ,Map.entry(3.35,3560.0)
         ,Map.entry(3.51,3570.0)
         ,Map.entry(3.81,3580.0)
         ,Map.entry(3.96,4000.0)

        );
        
}

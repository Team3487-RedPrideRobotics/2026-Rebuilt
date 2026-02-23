package frc.robot.generated;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;


public class LimelightConstants {
    //limelight-chassis/limelight-lront
    public static final Pose3d limelightFrontPose = new Pose3d(Units.inchesToMeters(-0.58)
                                                        ,  Units.inchesToMeters(-7.5) 
                                                        ,  Units.inchesToMeters(18)
                                                        ,  new Rotation3d());
    public static final String LimelightFrontID =  "limelight-chassis";
    //limelight-left
    public static final Pose3d limelightShooterOffset = new Pose3d(Units.inchesToMeters(-5.75)
                                                        ,  Units.inchesToMeters(1.) 
                                                        ,  Units.inchesToMeters(1.0)
                                                        ,  new Rotation3d());
    public static final Pose3d limelightShooterCenter = new Pose3d(Units.inchesToMeters(1.0)
                                                        ,  Units.inchesToMeters(1.0) 
                                                        ,  Units.inchesToMeters(1.0)
                                                        ,  new Rotation3d());
    public static final String LimelightShooterID =  "limelight-Shooter";
        

    //Global pose of the hubs
    public static final Pose2d BlueHubPose2d = new Pose2d(4.6228,4.034536,new Rotation2d(0));
    public static final Pose2d RedHubPose2d = new Pose2d(11.9126,4.034536,new Rotation2d(0));


    //Interpolator values 
    //WARNING: All current values do not actually represent the current robot's data!

    //Hood interpolator; distance to goal in (meters), pitch out (degrees)
    public static final InterpolatingDoubleTreeMap TurretHoodInterpolatorDEG = InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.13, 60.0)
         ,Map.entry(4.57,56.0)
         ,Map.entry(4.87,52.0)
        );


    //Flywheel Interpolator; distance to goal(meters), Speed out (RPM)
    public static final InterpolatingDoubleTreeMap TurretFlywheelInterpolatorRPM = InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.13, 2800.0)
         ,Map.entry(2.28,2800.0)
         ,Map.entry(2.43,2900.0)
         ,Map.entry(2.49,2900.0)
         ,Map.entry(2.74,3000.0)
         ,Map.entry(3.0,3000.0)
         ,Map.entry(3.2,3050.0)
         ,Map.entry(3.35,3100.0)
         ,Map.entry(3.51,3200.0)
         ,Map.entry(3.81,3200.0)
         ,Map.entry(3.96,3300.0)
         ,Map.entry(4.11,3300.0)
         ,Map.entry(4.27,3400.0)
         ,Map.entry(4.87,3400.0)
        );

    public static final InterpolatingDoubleTreeMap InverseTurretFlywheelInterpolatorRPM = InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2800.0,2.13)
         ,Map.entry(2800.0,2.28)
         ,Map.entry(2900.0,2.43)
         ,Map.entry(2900.0,2.49)
         ,Map.entry(3000.0,2.74)
         ,Map.entry(3000.0,3.0)
         ,Map.entry(3050.0,3.2)
         ,Map.entry(3100.0,3.35)
         ,Map.entry(3200.0,3.51)
         ,Map.entry(3200.0,3.81)
         ,Map.entry(3300.0,3.96)
         ,Map.entry(3300.0,4.11)
         ,Map.entry(3400.0,4.27)
         ,Map.entry(3400.0,4.87)
        );
        
}

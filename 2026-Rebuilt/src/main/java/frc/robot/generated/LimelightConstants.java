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
    public static final Pose3d limelightShooterOffset = new Pose3d(Units.inchesToMeters(6.76)
                                                        ,  Units.inchesToMeters(0.44) 
                                                        ,  Units.inchesToMeters(4.1)
                                                        ,  new Rotation3d());
    public static final Pose3d limelightShooterCenter = new Pose3d(Units.inchesToMeters(-5.75)
                                                        ,  Units.inchesToMeters(-4) 
                                                        ,  Units.inchesToMeters(17)
                                                        ,  new Rotation3d());
    public static final String LimelightShooterID =  "limelight-shooter";
        

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
          Map.entry(2.13, 3000.0)
         ,Map.entry(2.28,3000.0)
         ,Map.entry(2.43,3000.0)
         ,Map.entry(2.49,3000.0)
         ,Map.entry(2.74,3050.0)
         ,Map.entry(3.0,3100.0)
         ,Map.entry(3.2,3150.0)
         ,Map.entry(3.35,3200.0)
         ,Map.entry(3.51,3250.0)
         ,Map.entry(3.81,3300.0)
         ,Map.entry(3.96,3350.0)
         //,Map.entry(4.11,3300.0)
         //,Map.entry(4.27,3400.0)
         //,Map.entry(4.87,3400.0)
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

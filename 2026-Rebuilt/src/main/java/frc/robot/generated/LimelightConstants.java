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
    public static final Pose3d limelightFrontPose = new Pose3d(Units.inchesToMeters(-3)
                                                        ,  Units.inchesToMeters(10) 
                                                        ,  Units.inchesToMeters(12)
                                                        ,  new Rotation3d(0, 45, 90));
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
          Map.entry(2.13, 4100.0)
         ,Map.entry(2.28,4100.0)
         ,Map.entry(2.43,4100.0)
         ,Map.entry(2.49,4100.0)
         ,Map.entry(2.74,4150.0)
         ,Map.entry(3.0,4200.0)
         ,Map.entry(3.2,4250.0)
         ,Map.entry(3.35,4300.0)
         ,Map.entry(3.51,4350.0)
         ,Map.entry(3.81,4400.0)
         ,Map.entry(3.96,4450.0)

        );
        
}

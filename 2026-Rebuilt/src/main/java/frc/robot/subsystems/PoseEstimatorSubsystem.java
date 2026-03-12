package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.LimelightConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class PoseEstimatorSubsystem extends SubsystemBase{

    Limelight limelightFront;
    Limelight limelightShooter;

    Pigeon2   m_gyro;
    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    double shooterRotation2d;
    double shooterDegPerSecond;
    double shooterRotationalRate;

    Field2d m_field;
    Pose2d  m_robotPose2d = new Pose2d(0.0,0.0, new Rotation2d(0.0));
    Rotation3d robotRotation = new Rotation3d();

    boolean   tooFast;
    Optional<Pose2d> tempPose;
    double shooterSeenAprilTags;
    double chassisSeenAprilTags;

    LimelightPoseEstimator shooterPoseEstimator;
    LimelightPoseEstimator frontPoseEstimator;

    Optional<PoseEstimate> visionEstimateFront;
    Optional<PoseEstimate> visionEstimateShooter;

    public Field2d getField2d(){
        return m_field;
    }

    public Pose2d getRobotPose2d(){
        return(m_robotPose2d);
    }

    public void putShooterRotation(double shooterTurretAngle){
        shooterRotation2d = shooterTurretAngle;
    }

    public void putShooterRotationalVelocity(double shooterTurretDegPerSecond){
        shooterDegPerSecond = shooterTurretDegPerSecond;
    }

    public CommandSwerveDrivetrain getDrivetrain(){
        return m_CommandSwerveDrivetrain;
    }

    public void setThermalManagement(boolean ThermalManagement){
        if(ThermalManagement){limelightShooter.getSettings().withPipelineIndex(9).save();;}
        else{limelightShooter.getSettings().withPipelineIndex(0).save();}
    }


    public PoseEstimatorSubsystem(CommandSwerveDrivetrain MySillyLittleDrivetrain){
        limelightFront = new Limelight(LimelightConstants.LimelightFrontID);
        limelightShooter = new Limelight(LimelightConstants.LimelightShooterID);
        m_gyro = new Pigeon2(13);
        m_CommandSwerveDrivetrain = MySillyLittleDrivetrain;
        m_field = new Field2d();
        SmartDashboard.putData("Field",m_field);
        m_field.getObject("turretPose").setPose(Pose2d.kZero);
        m_field.getObject("BlueHub").setPose(Pose2d.kZero);
        m_field.getObject("RedHub").setPose(Pose2d.kZero);

        limelightFront.getSettings().withCameraOffset(LimelightConstants.limelightFrontPose).save();  

        shooterPoseEstimator = limelightShooter.createPoseEstimator(EstimationMode.MEGATAG2);
        frontPoseEstimator = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
        
    }

    @Override
    public void periodic() {

    BaseStatusSignal.refreshAll(m_gyro.getAngularVelocityXWorld(),
                                m_gyro.getAngularVelocityYWorld(),
                                m_gyro.getAngularVelocityZWorld(),
                                m_gyro.getRoll(),
                                m_gyro.getYaw(),
                                m_gyro.getPitch());

    robotRotation = new Rotation3d(
                    Degrees.of(m_gyro.getRoll().getValueAsDouble()),
                    Degrees.of(m_gyro.getPitch().getValueAsDouble()),
                    Degrees.of(m_gyro.getYaw().getValueAsDouble()));
    
    visionEstimateShooter = shooterPoseEstimator.getPoseEstimate();
    visionEstimateFront = frontPoseEstimator.getPoseEstimate();

    limelightShooter.getSettings().withCameraOffset(LimelightConstants.limelightShooterOffset.rotateAround(
                                 LimelightConstants.limelightShooterCenter.getTranslation()
                                ,new Rotation3d(Rotation2d.fromDegrees(shooterRotation2d))));
    
    //Update each of the limelights with the current robot orientation
    limelightFront.getSettings().withRobotOrientation(new Orientation3d(robotRotation,
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble())))).save();
    limelightShooter.getSettings().withRobotOrientation(new Orientation3d(robotRotation,
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble())))).save();
    // If the pose is present
    //visionEstimateFront.ifPresent((PoseEstimate poseEstimateFront) -> {
    // Add it to the pose estimator.
    //check if you can actually see tags
    //if(poseEstimateFront.tagCount >0 ){
    //m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateFront.pose.toPose2d(), poseEstimateFront.timestampSeconds);
    //}
    //});

    visionEstimateShooter.ifPresent((PoseEstimate poseEstimateShooter) -> {
        if(poseEstimateShooter.tagCount >1 ){
            if(poseEstimateShooter.pose.toPose2d() != Pose2d.kZero){
        m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateShooter.pose.toPose2d(), poseEstimateShooter.timestampSeconds);
    }}
    });
    

    Optional<Pose2d> tempPose = m_CommandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

    if (tempPose.isPresent()) {
        m_robotPose2d = tempPose.get();
    }

    m_field.setRobotPose(getRobotPose2d());
    m_field.getObject("BlueHub").setPose(LimelightConstants.BlueHubPose2d);
    m_field.getObject("RedHub").setPose(LimelightConstants.RedHubPose2d);
    m_field.getObject("turretPose").setPose(new Pose2d(getRobotPose2d().getTranslation(),getRobotPose2d().getRotation().plus(Rotation2d.fromDegrees(shooterRotation2d))));
}

}
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

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
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class PoseEstimatorSubsystem extends SubsystemBase{

    Limelight limelightFront;
    Limelight limelightShooter;

    Pigeon2   m_gyro;
    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    double shooterRotation2d;

    Field2d m_field;
    Optional<Pose2d> tempPose;
    Pose2d  m_robotPose2d = new Pose2d(0.0,0.0, new Rotation2d(0.0));

    boolean   tooFast;

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

    public CommandSwerveDrivetrain getDrivetrain(){
        return m_CommandSwerveDrivetrain;
    }


    public PoseEstimatorSubsystem(CommandSwerveDrivetrain MySillyLittleDrivetrain){
        limelightFront = new Limelight(LimelightConstants.LimelightFrontID);
        limelightShooter = new Limelight(LimelightConstants.LimelightShooterID);
        m_gyro = new Pigeon2(13);
        m_CommandSwerveDrivetrain = MySillyLittleDrivetrain;
        m_field = new Field2d();
        SmartDashboard.putData("Field",m_field);

        limelightFront.getSettings().withCameraOffset(LimelightConstants.limelightFrontPose).save();  
        
            //get the pose estimates from the limelights
        visionEstimateFront = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
        visionEstimateShooter = limelightShooter.createPoseEstimator(EstimationMode.MEGATAG2).getPoseEstimate();
    }

    @Override
    public void periodic() {

    limelightShooter.getSettings().withCameraOffset(LimelightConstants.limelightShooterOffset.rotateAround(
                                 LimelightConstants.limelightShooterCenter.getTranslation()
                                ,new Rotation3d(0,0,shooterRotation2d)));
    
    //Update each of the limelights with the current robot orientation
    limelightFront.getSettings().withRobotOrientation(new Orientation3d(m_gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();
    limelightShooter.getSettings().withRobotOrientation(new Orientation3d(m_gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();


    // If the pose is present
    visionEstimateFront.ifPresent((PoseEstimate poseEstimateFront) -> {
    // Add it to the pose estimator.
    m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateFront.pose.toPose2d(), poseEstimateFront.timestampSeconds);
    });
    visionEstimateShooter.ifPresent((PoseEstimate poseEstimateShooter) -> {
    m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateShooter.pose.toPose2d(), poseEstimateShooter.timestampSeconds);
    });

    Optional<Pose2d> tempPose = m_CommandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

    if (tempPose.isPresent()) {
        m_robotPose2d = tempPose.get();
    }

    m_field.setRobotPose(getRobotPose2d());
}
    


}
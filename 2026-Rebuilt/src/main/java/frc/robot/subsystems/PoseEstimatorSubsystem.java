package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.LimelightConstants;
import frc.robot.generated.SubsystemConstants;
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

    LimelightSim limelightFrontSim;
    LimelightSim limelightShooterSim;

    Pigeon2   m_gyro;
    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    double shooterRotation2d;
    double shooterDegPerSecond;
    double shooterRotationalRate;

    Field2d m_field;
    Pose2d  m_robotPose2d = new Pose2d(0.0,0.0, new Rotation2d(0.0));
    Rotation3d robotRotation = new Rotation3d();
    Matrix<N3,N1> shooterStddevs = VecBuilder.fill(0.1,0.1,0.1);
    Matrix<N3,N1> chassisStddevs = VecBuilder.fill(0.5,0.5,0.5);
    boolean visionEstimatesEnabled = true;
    boolean isSimulation = RobotBase.isSimulation();

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
        if(ThermalManagement){limelightShooter.getSettings().withPipelineIndex(9).save();}
        else{limelightShooter.getSettings().withPipelineIndex(0).save();}
    }

    public void setVisionEstimatesEnabled(){
        visionEstimatesEnabled = !visionEstimatesEnabled;
    }


    public PoseEstimatorSubsystem(CommandSwerveDrivetrain MySillyLittleDrivetrain){        
        if(isSimulation){
            limelightFrontSim = new LimelightSim(LimelightConstants.LimelightFrontID,AprilTagFields.k2026RebuiltAndymark);
            limelightShooterSim = new LimelightSim(LimelightConstants.LimelightShooterID,AprilTagFields.k2026RebuiltAndymark);
            limelightFrontSim.setCameraSimOutput(true, true);
            limelightShooterSim.setCameraSimOutput(true, true);
        }

        limelightFront = new Limelight(LimelightConstants.LimelightFrontID);
        limelightShooter = new Limelight(LimelightConstants.LimelightShooterID);
        
        m_gyro = new Pigeon2(13);

        m_CommandSwerveDrivetrain = MySillyLittleDrivetrain;

        m_field = new Field2d();

        SmartDashboard.putData("Field",m_field);
        m_field.getObject("ShooterPoseEstimate").setPose(Pose2d.kZero);
        m_field.getObject("turretPose").setPose(Pose2d.kZero);
        m_field.getObject("BlueHub").setPose(Pose2d.kZero);
        m_field.getObject("RedHub").setPose(Pose2d.kZero);

        limelightFront.getSettings().withCameraOffset(LimelightConstants.limelightFrontPose).save();
        limelightShooter.getSettings().withCameraOffset(LimelightConstants.limelightShooterPose).save();

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

    //Update each of the limelights with the current robot orientation
    limelightFront.getSettings().withRobotOrientation(new Orientation3d(robotRotation,
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble())))).save();
    limelightShooter.getSettings().withRobotOrientation(new Orientation3d(robotRotation,
												 new AngularVelocity3d(DegreesPerSecond.of(m_gyro.getAngularVelocityXWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityYWorld().getValueAsDouble()),
																	   DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble())))).save();
    
    if(!DriverStation.isAutonomousEnabled()){
    // If the pose is present
    visionEstimateFront.ifPresent((PoseEstimate poseEstimateFront) -> {
    if(poseEstimateFront.tagCount >0 ){
        Matrix<N3,N1> CurrentStdvs = chassisStddevs.times(Math.pow(!isSimulation?poseEstimateFront.getAvgTagAmbiguity():1.05+1,5));
        if(poseEstimateFront.pose.toPose2d() != Pose2d.kZero){
                if(DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble()).in(DegreesPerSecond)<10){
    if(visionEstimatesEnabled){m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateFront.pose.toPose2d(), poseEstimateFront.timestampSeconds,CurrentStdvs);}
    }}}
    });

    visionEstimateShooter.ifPresent((PoseEstimate poseEstimateShooter) -> {
        if(poseEstimateShooter.tagCount >0){
            Matrix<N3,N1> CurrentStdvs = shooterStddevs.times(Math.pow(!isSimulation?poseEstimateShooter.getAvgTagAmbiguity():1.05+1,5));
            if(poseEstimateShooter.pose.toPose2d() != Pose2d.kZero){
                if(DegreesPerSecond.of(m_gyro.getAngularVelocityZWorld().getValueAsDouble()).in(DegreesPerSecond)<10){
        if(visionEstimatesEnabled){m_CommandSwerveDrivetrain.addVisionMeasurement(poseEstimateShooter.pose.toPose2d(), poseEstimateShooter.timestampSeconds,CurrentStdvs);}
        m_field.getObject("ShooterPoseEstimate").setPose(poseEstimateShooter.pose.toPose2d());
    }}}
    });
    
    }
    Optional<Pose2d> tempPose = m_CommandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

    if (tempPose.isPresent()) {
        m_robotPose2d = tempPose.get();
    }

    m_field.setRobotPose(getRobotPose2d());
    m_field.getObject("BlueHub").setPose(LimelightConstants.BlueHubPose2d);
    m_field.getObject("RedHub").setPose(LimelightConstants.RedHubPose2d);
    m_field.getObject("turretPose").setPose(new Pose2d(getRobotPose2d().getTranslation(),getRobotPose2d().getRotation().plus(Rotation2d.fromDegrees(shooterRotation2d))));
}

@Override
public void simulationPeriodic() {
    limelightFrontSim.updateBotPose(m_robotPose2d);
    limelightShooterSim.updateBotPose(m_robotPose2d);
}

}
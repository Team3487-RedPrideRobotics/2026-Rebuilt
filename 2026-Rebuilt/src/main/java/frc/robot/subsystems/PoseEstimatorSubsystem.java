package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.generated.VisionConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class PoseEstimatorSubsystem extends SubsystemBase{

    PhotonCamera llCameraLeft;
    PhotonCamera llCameraRight;

    PhotonCameraSim llCameraLeftSim;
    PhotonCameraSim llCameraRightSim;
    VisionSystemSim VisionSystemSim;

    List<PhotonPipelineResult> llCameraLeftResults;
    List<PhotonPipelineResult> llCameraRightResults;

    PhotonPoseEstimator llCameraLeftPoseEstimator;
    PhotonPoseEstimator llCameraRightPoseEstimator;

    Optional<EstimatedRobotPose> m_estimatedRobotPose;
    
    CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    double shooterRotation2d;
    double shooterDegPerSecond;
    double shooterRotationalRate;

    Field2d m_field;
    Pose2d  m_robotPose2d = new Pose2d(0.0,0.0, new Rotation2d(0.0)); 
    Rotation3d robotRotation = new Rotation3d();
    boolean visionEstimatesEnabled = true;
    boolean isSimulation = RobotBase.isSimulation();

    boolean          tooFast;
    List<Pose2d>     poseEstimates;
    Optional<Pose2d> tempPose;

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
        if(ThermalManagement){
            llCameraLeft.setFPSLimit(4);
            llCameraRight.setFPSLimit(4);
            System.out.println("Vision thermal managment enabled!");
        }
        else{
            llCameraLeft.setFPSLimit(-1);
            llCameraRight.setFPSLimit(-1);
            System.out.println("Vision thermal managment disabled!");
        }
    }

    public void setVisionEstimatesEnabled(boolean enabled){
        visionEstimatesEnabled = enabled;
        if(visionEstimatesEnabled){System.out.println("VisionEnabled!");}
        else{System.out.println("VisionDisaibled!");}
    }

    public void toggleVisionEstimatesEnabled(){
        visionEstimatesEnabled = !visionEstimatesEnabled;
        if(visionEstimatesEnabled){System.out.println("VisionEnabled!");}
        else{System.out.println("VisionDisaibled!");}
    }

    public Matrix<N3,N1> findEstimatedStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){
        Matrix<N3,N1> StdDevs;

        //set return to default
        StdDevs = VisionConstants.SingleTagStdDevs;
        if(estimatedPose.isEmpty()) return StdDevs; //return default if pose doesnt exist somehow

        int   tagCount= 0;
        float avgDist = 0;
        for (var tgt : targets) { //find average distance to each tag
                var tagPose = llCameraLeftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                tagCount++;
                avgDist +=
                        tagPose.get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

        if (tagCount<2) return StdDevs;
        if (tagCount>1) StdDevs = VisionConstants.MultiTagStdDevs;
        StdDevs.times(avgDist*avgDist/tagCount);
        
        return StdDevs;
    }

    public PoseEstimatorSubsystem(CommandSwerveDrivetrain MySillyLittleDrivetrain){        
        
        llCameraLeft = new PhotonCamera(VisionConstants.llProcessorLeftID);
        llCameraRight = new PhotonCamera(VisionConstants.llProcessorRightID);

        llCameraLeftPoseEstimator = new PhotonPoseEstimator(VisionConstants.AprilTagLayout, VisionConstants.llCameraLeftPose);
        llCameraRightPoseEstimator= new PhotonPoseEstimator(VisionConstants.AprilTagLayout, VisionConstants.llCameraRightPose);

        if(Robot.isSimulation()){
            llCameraLeftSim = new PhotonCameraSim(llCameraLeft, VisionConstants.llCameraSimProperties);
            llCameraRightSim = new PhotonCameraSim(llCameraRight, VisionConstants.llCameraSimProperties);
            VisionSystemSim = new VisionSystemSim("main");
            VisionSystemSim.addAprilTags(VisionConstants.AprilTagLayout);
            VisionSystemSim.addCamera(llCameraLeftSim, VisionConstants.llCameraLeftPose);
            VisionSystemSim.addCamera(llCameraRightSim, VisionConstants.llCameraRightPose);
            llCameraLeftSim.enableDrawWireframe(true);
            llCameraRightSim.enableDrawWireframe(true);
        }

        m_CommandSwerveDrivetrain = MySillyLittleDrivetrain;

        m_field = new Field2d();

        poseEstimates = new ArrayList<>(List.of());

        SmartDashboard.putData("Field",m_field);
        m_field.getObject("ShooterPoseEstimate").setPose(Pose2d.kZero);
        m_field.getObject("PoseEstimates").setPoses();
        m_field.getObject("TurretPose").setPose(Pose2d.kZero);
        m_field.getObject("BlueHub").setPose(Pose2d.kZero);
        m_field.getObject("RedHub").setPose(Pose2d.kZero);
        
    }

    @Override
    public void periodic() {

    poseEstimates.clear(); //clear pose estimate display
    
    //get results from cameras
    llCameraLeftResults  = llCameraLeft.getAllUnreadResults();
    llCameraRightResults = llCameraRight.getAllUnreadResults();

    //clear estimate
    m_estimatedRobotPose = Optional.empty();
    //cycle through all results
    for(var result : llCameraLeftResults){
        m_estimatedRobotPose = llCameraLeftPoseEstimator.estimateCoprocMultiTagPose(result); //check for multitag
        if(m_estimatedRobotPose.isEmpty()) llCameraLeftPoseEstimator.estimateLowestAmbiguityPose(result); //if no multitag, take single

        m_estimatedRobotPose.ifPresent(estimate -> { //if estimate exists
            poseEstimates.add(estimate.estimatedPose.toPose2d()); //add estimate to display
            //add vision esimates if enabled
            if(visionEstimatesEnabled)m_CommandSwerveDrivetrain.addVisionMeasurement(estimate.estimatedPose.toPose2d(),estimate.timestampSeconds,findEstimatedStdDevs(m_estimatedRobotPose, result.getTargets()));
        }); 
    }

    //clear estimate
    m_estimatedRobotPose = Optional.empty();
    //cycle through all results
    for(var result : llCameraRightResults){
        m_estimatedRobotPose = llCameraRightPoseEstimator.estimateCoprocMultiTagPose(result); //check for multitag
        if(m_estimatedRobotPose.isEmpty()) llCameraRightPoseEstimator.estimateLowestAmbiguityPose(result); //if no multitag, take single

        m_estimatedRobotPose.ifPresent(estimate -> { //if estimate exists
            poseEstimates.add(estimate.estimatedPose.toPose2d()); //add estimate to display
            // add vision estimates if enabled
            if(visionEstimatesEnabled)m_CommandSwerveDrivetrain.addVisionMeasurement(estimate.estimatedPose.toPose2d(),estimate.timestampSeconds,findEstimatedStdDevs(m_estimatedRobotPose, result.getTargets()));
        }); 
    }

    Optional<Pose2d> tempPose = m_CommandSwerveDrivetrain.samplePoseAt(Utils.getCurrentTimeSeconds());

    tempPose.ifPresent((Pose2d pose)->{
        m_robotPose2d = pose;
    });

    //Update field objects
    m_field.setRobotPose(getRobotPose2d());
    m_field.getObject("PoseEstimates").setPoses(poseEstimates);
    m_field.getObject("BlueHub").setPose(VisionConstants.BlueHubPose2d);
    m_field.getObject("RedHub").setPose(VisionConstants.RedHubPose2d);
    m_field.getObject("turretPose").setPose(new Pose2d(getRobotPose2d().getTranslation(),getRobotPose2d().getRotation().plus(Rotation2d.fromDegrees(shooterRotation2d))));
}

@Override
public void simulationPeriodic(){
    VisionSystemSim.update(m_robotPose2d);
}
}

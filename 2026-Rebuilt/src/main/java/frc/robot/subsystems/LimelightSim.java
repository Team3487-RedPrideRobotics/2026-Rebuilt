package frc.robot.subsystems;

import java.util.Arrays;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSim extends SubsystemBase{

    private AprilTagFieldLayout m_AprilTagFieldLayout;

    private String myString;
    private String OutputString;
    private int m_pipeline;

    private Pose3d botPose;
    private Pose3d m_robotPose;

    private PhotonCamera m_camera;
    private PhotonCameraSim m_cameraSim;

    private VisionSystemSim m_VisionSystem;

    private PhotonPipelineResult m_cameraResult;

    public LimelightSim(String inputString, AprilTagFields AprilTagField){
        OutputString = inputString;
        myString = inputString + "Sim";
        m_pipeline = 0;

        m_AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagField);


        m_camera = new PhotonCamera(myString+"Cam");
        m_cameraSim = new PhotonCameraSim(m_camera, SimCameraProperties.LL2_640_480(),m_AprilTagFieldLayout);
        m_VisionSystem = new VisionSystemSim(myString);

        m_VisionSystem.addCamera(m_cameraSim, Transform3d.kZero);
        botPose = Pose3d.kZero;
        m_cameraResult = new PhotonPipelineResult();

        m_VisionSystem.addAprilTags(m_AprilTagFieldLayout);

        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("getpipe").setInteger(0);
    }

    double[] constructBotposeOutputArray(){
        double[] outputDoubleArray = new double[11];
        outputDoubleArray[0] = botPose.getX();
        outputDoubleArray[1] = botPose.getY();
        outputDoubleArray[2] = botPose.getZ();
        outputDoubleArray[3] = Units.radiansToDegrees(botPose.getRotation().getX());
        outputDoubleArray[4] = Units.radiansToDegrees(botPose.getRotation().getY());
        outputDoubleArray[5] = Units.radiansToDegrees(botPose.getRotation().getZ());
        outputDoubleArray[6] = SimCameraProperties.LL2_640_480().estLatencyMs();
        outputDoubleArray[7] = m_cameraResult.targets.size();
        outputDoubleArray[8] = 0;
        outputDoubleArray[9] = m_cameraResult.getBestTarget()!=null?m_cameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm():0;
        outputDoubleArray[10]= m_cameraResult.getBestTarget()!=null?m_cameraResult.getBestTarget().getArea():0;
        return outputDoubleArray;
    }

    double[] constructRawFidicuals(){
        double[] outputDoubleArray;
        if(!m_cameraResult.hasTargets()){
            outputDoubleArray = new double[7];
            outputDoubleArray[0] = 0;
            outputDoubleArray[1] = 0;
            outputDoubleArray[2] = 0;
            outputDoubleArray[3] = 0;
            outputDoubleArray[4] = 0;
            outputDoubleArray[5] = 0;
            outputDoubleArray[6] = 1;
            return outputDoubleArray;
        }
        outputDoubleArray = new double[m_cameraResult.getTargets().size()*7];
        for(int fiducial = 1; fiducial < m_cameraResult.getTargets().size()+1;fiducial+=1){
            outputDoubleArray[fiducial*7-7] = m_cameraResult.getBestTarget().getFiducialId();
            outputDoubleArray[fiducial*7-6] = m_cameraResult.getBestTarget().getYaw()/82*640;
            outputDoubleArray[fiducial*7-5] = m_cameraResult.getBestTarget().getPitch()/52*480;
            outputDoubleArray[fiducial*7-4] = m_cameraResult.getBestTarget().getArea();
            outputDoubleArray[fiducial*7-3] = m_cameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            outputDoubleArray[fiducial*7-2] = m_cameraResult.getBestTarget().getBestCameraToTarget().getTranslation().getDistance(updateCameraPos().getTranslation());
            outputDoubleArray[fiducial*7-1] = m_cameraResult.getBestTarget().getPoseAmbiguity()>=0?m_cameraResult.getBestTarget().getPoseAmbiguity():1; 
        }
        return outputDoubleArray;
    }

    Field2d getField2dImage(){
        return m_VisionSystem.getDebugField();
    }

    void setCameraSimOutput(boolean camEnabled, boolean fieldEnabled){
        m_cameraSim.enableRawStream(camEnabled);
        m_cameraSim.enableProcessedStream(camEnabled);
        m_cameraSim.enableDrawWireframe(fieldEnabled);
    }

    Pose3d updateCameraPos(){ //update the camera's pose relative to the robot
        double[] cameraPoseDoubleArray = NetworkTableInstance.getDefault().getTable(OutputString).getEntry("camerapose_robotspace_set").getDoubleArray(new double[6]);
        Transform3d cameraPose = new Transform3d(cameraPoseDoubleArray[0],cameraPoseDoubleArray[1],cameraPoseDoubleArray[2],
                                  new Rotation3d(Units.degreesToRadians(cameraPoseDoubleArray[3]),Units.degreesToRadians(cameraPoseDoubleArray[4]),Units.degreesToRadians(cameraPoseDoubleArray[5])));
        m_VisionSystem.adjustCamera(m_cameraSim, cameraPose);
        return (new Pose3d(new Translation3d(cameraPose.getTranslation().getX(),cameraPose.getTranslation().getY(),cameraPose.getTranslation().getZ()), cameraPose.getRotation()));
    }

    void updateOutputPoses(){
        //update megatag 1 & 2
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("botpose").setDoubleArray(constructBotposeOutputArray());
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("botpose_wpiblue").setDoubleArray(constructBotposeOutputArray());
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("botpose_orb").setDoubleArray(constructBotposeOutputArray());
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("botpose_orb_wpiblue").setDoubleArray(constructBotposeOutputArray());
        //the best viewed tag's Id
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("tid").setInteger(m_cameraResult.getBestTarget()!=null?m_cameraResult.getBestTarget().getFiducialId():-1);
        //raw fudicials        
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("rawfiducials").setDoubleArray(constructRawFidicuals());
    }

    void updateBotPose(Pose2d robotPose2d){
        m_robotPose = new Pose3d(robotPose2d);
    }

    @Override
    public void simulationPeriodic() {
        m_cameraResult = m_cameraSim.process(SimCameraProperties.LL2_640_480().estLatencyMs()
                                                , updateCameraPos() 
                                                , Arrays.asList(m_VisionSystem.getVisionTargets().toArray(new VisionTargetSim[0])));
        if(m_VisionSystem.getRobotPose().getX() != 0){botPose = m_VisionSystem.getRobotPose();}
        m_VisionSystem.update(m_robotPose);
        updateOutputPoses();
        NetworkTableInstance.getDefault().getTable(OutputString).getEntry("getpipe").setInteger(m_pipeline);
        m_pipeline = (int) NetworkTableInstance.getDefault().getTable(OutputString).getEntry("pipeline").getInteger(0);

    }
}
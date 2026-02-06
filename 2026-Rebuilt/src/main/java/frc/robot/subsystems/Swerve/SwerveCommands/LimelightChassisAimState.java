package frc.robot.subsystems.Swerve.SwerveCommands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightTargetData;
import limelight.networktables.Orientation3d;


public class LimelightChassisAimState extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private SwerveRequest.RobotCentric robotCentric;
    
    private Pigeon2 gyro;
    private Limelight limeLight;
    private LimelightTargetData targetData;

    final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    double apriltag;
    double turnRate;
    double xVelocity;
    double yVelocity;
    boolean done;

    Pose2d goalPose2d;
    //in m/s:
    double maxLimelightSpeed = TunerConstants.limelightToRobotMaxSpeed.in(MetersPerSecond);
    PIDController pidControllerX = TunerConstants.LIMELIGHT_PID_CONTROLLER_TRANS_X;
    PIDController pidControllerY = TunerConstants.LIMELIGHT_PID_CONTROLLER_TRANS_Y;
    PIDController pidControllerTheta = TunerConstants.LIMELIGHT_PID_CONTROLLER_ROTATION;
    //in meters:
    double limelightTolerance = 0.05;
    Pose3d RobotPose;



    public LimelightChassisAimState(CommandSwerveDrivetrain RealDescriptiveName, SwerveRequest.RobotCentric RobotCentric,Pose2d goal){

        m_Drivetrain  = RealDescriptiveName;
        addRequirements(RealDescriptiveName);
        robotCentric = RobotCentric;
        goalPose2d = goal;

    }

    @Override
    public void initialize(){
    gyro = new Pigeon2(13);
    limeLight = new Limelight("limelight-chassis");
    //TODO:Tune PID Values and Ensure that this system works      
    limeLight.getSettings().withCameraOffset(new Pose3d(0,0,0, new Rotation3d())).withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();
    targetData = new LimelightTargetData(limeLight);
    apriltag = targetData.getAprilTagID();
    
    boolean validTarget =  targetData.getTargetStatus();
    System.out.println("hello, this is a " + validTarget + " april tag = " + apriltag);
    done = false;
    }


    @Override
    public void execute(){
        limeLight.getSettings().withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getAngularVelocityXDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityZDevice().getValueAsDouble()),
																	   DegreesPerSecond.of(gyro.getAngularVelocityYDevice().getValueAsDouble())))).save();
        RobotPose = targetData.getRobotToTarget();

        if(apriltag != -1){
         
         xVelocity = MathUtil.clamp(-pidControllerX.calculate(goalPose2d.getX(),RobotPose.getX()),-maxLimelightSpeed,maxLimelightSpeed);
         yVelocity = MathUtil.clamp(-pidControllerY.calculate(goalPose2d.getY(),RobotPose.getZ()),-maxLimelightSpeed,maxLimelightSpeed);
         turnRate = MathUtil.clamp(pidControllerY.calculate(goalPose2d.getRotation().getRadians(),RobotPose.getRotation().getZ()),-1,1);
         
         if(Math.abs(goalPose2d.getX()-RobotPose.getX())<=limelightTolerance && Math.abs(goalPose2d.getY()-RobotPose.getY())<=limelightTolerance && Math.abs(goalPose2d.getRotation().getRadians()-RobotPose.getRotation().getX())<=0.2){
         m_Drivetrain.setControl(robotCentric.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(turnRate));
         }
         else{
         
         }
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("the robot pose at end: " + RobotPose);
        System.out.println("chassis speed out X: " + xVelocity + " Y: " + yVelocity + " Turn Rate: " + turnRate);
    }

    @Override
    public boolean isFinished(){
        return done;
    }

}

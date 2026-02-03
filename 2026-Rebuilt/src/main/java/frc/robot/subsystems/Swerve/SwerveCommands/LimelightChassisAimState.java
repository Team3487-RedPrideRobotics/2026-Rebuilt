package frc.robot.subsystems.Swerve.SwerveCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import limelight.*;
import limelight.networktables.LimelightTargetData;

public class LimelightChassisAimState extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private SwerveRequest.RobotCentric robotCentric;
    private RobotContainer robotContainer;
    
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
    double limelightTolerance = 0.1;
    Pose2d RobotPose;



    public LimelightChassisAimState(CommandSwerveDrivetrain RealDescriptiveName, SwerveRequest.RobotCentric RobotCentric,Pose2d goal){

        m_Drivetrain  = RealDescriptiveName;
        addRequirements(RealDescriptiveName);
        robotCentric = RobotCentric;
        goalPose2d = goal;

    }

    @Override
    public void initialize(){
    limeLight = new Limelight("limelight-chassis");
    targetData = new LimelightTargetData(limeLight);
    apriltag = targetData.getAprilTagID();
    System.out.println("hello, this is the april tag = " + apriltag);
    done = false;
    }


    @Override
    public void execute(){
        RobotPose = targetData.getRobotToTarget().toPose2d();
        
        
        
        if(apriltag != -1){
         
         xVelocity = MathUtil.clamp(-pidControllerX.calculate(goalPose2d.getX(),RobotPose.getX()),-maxLimelightSpeed,maxLimelightSpeed);
         yVelocity = MathUtil.clamp(-pidControllerY.calculate(goalPose2d.getY(),RobotPose.getY()),-maxLimelightSpeed,maxLimelightSpeed);
         turnRate = MathUtil.clamp(pidControllerY.calculate(0,RobotPose.getRotation().getDegrees()),-1,1);
         
         if(Math.abs(goalPose2d.getX()-RobotPose.getX())<=limelightTolerance && Math.abs(goalPose2d.getY()-RobotPose.getY())<=limelightTolerance && Math.abs(goalPose2d.getRotation().getDegrees()-RobotPose.getRotation().getDegrees())<=5){
         CommandScheduler.getInstance().schedule(m_Drivetrain.applyRequest(() -> robotCentric.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(turnRate)));
         }
         else{
         done = true;
         }
        }
        else{
         done = true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        //CommandScheduler.getInstance().schedule(m_Drivetrain.applyRequest(() -> idle));
        System.out.println("the robot pose at end: " + RobotPose);
    }

    @Override
    public boolean isFinished(){
        return done;
    }

}

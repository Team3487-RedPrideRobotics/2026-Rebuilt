package frc.robot.subsystems.Swerve.SwerveCommands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.MathUtil;

import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

import limelight.Limelight;
import limelight.networktables.target.AprilTagFiducial;

public class LimelightChassisAimState extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private SwerveRequest.RobotCentric robotCentric;
    private RobotContainer robotContainer;
    private static AprilTagFiducial apriltagTrack;

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

    Limelight limeLight = new Limelight("Limelight");

    public LimelightChassisAimState(CommandSwerveDrivetrain RealDescriptiveName, Pose2d goal){

        m_Drivetrain  = RealDescriptiveName;
        addRequirements(RealDescriptiveName);
        robotCentric = robotContainer.RobotCentricDrive;
        goalPose2d = goal;

    }

    @Override
    public void initialize(){
        apriltag = LimelightHelpers.getFiducialID("Limelight");        
    }


    @Override
    public void execute(){
        if(apriltag != 0){
         RobotPose = apriltagTrack.getTargetPose_RobotSpace2D();
         
         xVelocity = MathUtil.clamp(-pidControllerX.calculate(goalPose2d.getX(),RobotPose.getX()),-maxLimelightSpeed,maxLimelightSpeed);
         yVelocity = MathUtil.clamp(-pidControllerY.calculate(goalPose2d.getY(),RobotPose.getY()),-maxLimelightSpeed,maxLimelightSpeed);
         turnRate = MathUtil.clamp(pidControllerY.calculate(0,RobotPose.getRotation().getDegrees()),-1,1);
         
         if(Math.abs(goalPose2d.getX()-RobotPose.getX())<=limelightTolerance && Math.abs(goalPose2d.getY()-RobotPose.getY())<=limelightTolerance && Math.abs(goalPose2d.getRotation().getDegrees()-RobotPose.getRotation().getDegrees())<=limelightTolerance){
         CommandScheduler.getInstance().schedule(m_Drivetrain.applyRequest(() -> robotCentric.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(turnRate)));
         }
         else{
         done = true;
         }
        }
    }

     @Override
    public void end(boolean interrupted)
    {
        CommandScheduler.getInstance().schedule(m_Drivetrain.applyRequest(() -> idle));
    }

     @Override
    public boolean isFinished(){
        return done;
    }

}

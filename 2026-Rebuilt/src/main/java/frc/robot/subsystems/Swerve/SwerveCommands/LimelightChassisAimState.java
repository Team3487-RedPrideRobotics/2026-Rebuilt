package frc.robot.subsystems.Swerve.SwerveCommands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

public class LimelightChassisAimState extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private SwerveRequest.RobotCentric robotCentric;
    private RobotContainer robotContainer;

    double apriltag;
    double turnRate;
    double xVelocity;
    double yVelocity;
    boolean done;
    //in m/s:
    double maxLimelightSpeed = 0.5;
    //in meters:
    double limelightTolerance = 0.1;

    public LimelightChassisAimState(CommandSwerveDrivetrain RealDescriptiveName){

        m_Drivetrain  = RealDescriptiveName;
        addRequirements(RealDescriptiveName);
        robotCentric = robotContainer.RobotCentricDrive;

    }

    @Override
    public void initialize(){
        apriltag = LimelightHelpers.getFiducialID("Limelight");        
    }


    @Override
    public void execute(){

        if(false){
        CommandScheduler.getInstance().schedule(m_Drivetrain.applyRequest(() -> robotCentric.withVelocityX(xVelocity).withVelocityY(yVelocity).withRotationalRate(turnRate)));
        }

    }

     @Override
    public void end(boolean interrupted)
    {
        
    }

     @Override
    public boolean isFinished(){
        return done;
    }

}

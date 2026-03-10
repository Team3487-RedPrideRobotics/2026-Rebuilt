package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.LimelightConstants;
import frc.robot.subsystems.ShooterSubsystem.*;

public class BlueHoodAutoAimState extends Command {

    ShooterSubsystem subsystem;
    RobotContainer m_RobotContainer;

    double Position;

    boolean done;

    public BlueHoodAutoAimState( ShooterSubsystem Subsystem, RobotContainer m_RobotContainer){
        
        super();
        subsystem = Subsystem;
        addRequirements(Subsystem);
        this.m_RobotContainer = m_RobotContainer;

    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        
        if (subsystem.FullTurretAutoAim(m_RobotContainer.getInstance().IsRed ?LimelightConstants.RedHubPose2d : LimelightConstants.BlueHubPose2d , 0.1)) {
            System.out.println("Oh Im aimin' it! OwO");
            done = true;
        }
        else{
            done = false;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopHoodMotor();
        subsystem.StopTurretMotor();
    }

}
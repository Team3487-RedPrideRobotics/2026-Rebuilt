package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.generated.LimelightConstants;
import frc.robot.subsystems.ShooterSubsystem.*;

public class AutoHubAimState extends Command {

    ShooterSubsystem subsystem;
    RobotContainer m_RobotContainer;

    double Position;

    boolean done;

    public AutoHubAimState( ShooterSubsystem Subsystem, RobotContainer m_RobotContainer){
        
        super();
        subsystem = Subsystem;
        addRequirements(subsystem);
        this.m_RobotContainer = m_RobotContainer;

    }

    @Override
    public void initialize() {
        done = false;
        System.out.println("starting to aim");
    }

    @Override
    public void execute() {
        System.out.println("Aiming");
        if (subsystem.FullTurretAutoAim(m_RobotContainer.getInstance().IsRed ?LimelightConstants.RedHubPose2d : LimelightConstants.BlueHubPose2d)) {
            done = true;
        }
        else{
            done = false;
            System.out.println("Aiming but in the if statement ig");
        }
        
    }

    @Override
    public boolean isFinished() {
        System.out.println(done);
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopHoodMotor();
        subsystem.StopTurretMotor();
    }

}
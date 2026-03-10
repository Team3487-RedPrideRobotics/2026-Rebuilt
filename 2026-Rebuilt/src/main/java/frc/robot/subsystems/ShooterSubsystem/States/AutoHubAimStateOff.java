package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem.*;

public class AutoHubAimStateOff extends Command {

    ShooterSubsystem subsystem;
    RobotContainer m_RobotContainer;

    double Position;

    boolean done;

    public AutoHubAimStateOff( ShooterSubsystem Subsystem, RobotContainer m_RobotContainer){
        
        super();
        subsystem = Subsystem;
        addRequirements(subsystem);
        this.m_RobotContainer = m_RobotContainer;

    }

    @Override
    public void initialize() {
        done = false;
        System.out.println("Stopping aiming at hub");
        subsystem.continueousTurretAutoAim(false, m_RobotContainer.IsRed);
    }

    @Override
    public void execute() {
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
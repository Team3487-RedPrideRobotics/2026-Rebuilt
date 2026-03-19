package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem.*;

public class AutoHubAimStatePerm extends Command {

    ShooterSubsystem subsystem;
    RobotContainer m_RobotContainer;

    double Position;

    boolean done;

    public AutoHubAimStatePerm( ShooterSubsystem Subsystem, RobotContainer m_RobotContainer){
        
        super();
        subsystem = Subsystem;
        addRequirements(subsystem);
        this.m_RobotContainer = m_RobotContainer;

    }

    @Override
    public void initialize() {
        done = false;
        subsystem.continueousTurretAutoAim(true, m_RobotContainer.IsRed);
    }
}
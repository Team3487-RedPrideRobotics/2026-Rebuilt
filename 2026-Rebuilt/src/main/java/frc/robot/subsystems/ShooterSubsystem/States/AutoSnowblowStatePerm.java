package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterSubsystem.*;

public class AutoSnowblowStatePerm extends Command {

    ShooterSubsystem subsystem;
    RobotContainer m_RobotContainer;

    double Position;

    boolean done;

    public AutoSnowblowStatePerm( ShooterSubsystem Subsystem, RobotContainer m_RobotContainer){
        
        super();
        subsystem = Subsystem;
        addRequirements(subsystem);
        this.m_RobotContainer = m_RobotContainer;

    }

    @Override
    public void initialize() {
        done = false;
        System.out.println("starting to aim to snowblow");
    }

    @Override
    public void execute() {
        subsystem.continueousTurretSlowblowAim(true, m_RobotContainer.IsRed);
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
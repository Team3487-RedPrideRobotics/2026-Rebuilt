package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class FlywheelIdleState extends Command {

    ShooterSubsystem subsystem;


    public FlywheelIdleState( ShooterSubsystem Subsystem){
        
        addRequirements(Subsystem);
        subsystem = Subsystem;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunFlywheelMotor(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
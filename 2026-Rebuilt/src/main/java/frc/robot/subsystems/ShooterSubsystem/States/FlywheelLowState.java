package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class FlywheelLowState extends Command {

    ShooterSubsystem subsystem;


    public FlywheelLowState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
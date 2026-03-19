package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class FlywheelIdleStatePerm extends Command {

    ShooterSubsystem subsystem;

    boolean done;


    public FlywheelIdleStatePerm( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        done = false;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunFlywheelMotor(3000/60);
    }

    @Override
    public void end(boolean interrupted) {
    }

}
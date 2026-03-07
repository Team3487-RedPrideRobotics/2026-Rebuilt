package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class FlywheelOffState extends Command {

    ShooterSubsystem subsystem;

    public FlywheelOffState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;

    }

    @Override
    public void execute() {
        subsystem.StopFlywheelMotors();
    }
}
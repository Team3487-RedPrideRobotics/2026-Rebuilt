package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class FlywheelHighState extends Command {

    ShooterSubsystem subsystem;


    public FlywheelHighState( ShooterSubsystem Subsystem){
        
        super();
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunFlywheelMotor(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopFlywheelMotors();
    }

}
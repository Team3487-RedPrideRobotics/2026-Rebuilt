package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class HoodDownState extends Command {

    ShooterSubsystem subsystem;


    public HoodDownState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        //subsystem.HoodPID(0, 1, 1, 0.1);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopHoodMotor();
    }

}
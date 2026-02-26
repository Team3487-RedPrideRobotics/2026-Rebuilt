package frc.robot.subsystems.KickerSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem.kickerSubsystem;

public class KickerReverseState extends Command {
    
    kickerSubsystem subsystem;


    public KickerReverseState(kickerSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(-1);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopMotors();

    }

}

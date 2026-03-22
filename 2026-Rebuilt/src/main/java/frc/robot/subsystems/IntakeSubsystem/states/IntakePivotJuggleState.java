package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakePivotJuggleState extends Command {
        
IntakeSubsystem subsystem;

    boolean done;

    public IntakePivotJuggleState(IntakeSubsystem Subsystem){

        subsystem = Subsystem;
        addRequirements(Subsystem);

    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        done = subsystem.IntakePiviotPID(45, 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopMotorPivot();
        System.err.println("finished juggle");
    }

    
    @Override
    public boolean isFinished() {
        return done;
    }
    
}

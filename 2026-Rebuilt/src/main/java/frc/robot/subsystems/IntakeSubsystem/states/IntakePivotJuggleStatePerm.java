package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakePivotJuggleStatePerm extends Command {
        
IntakeSubsystem subsystem;

    boolean done;

    public IntakePivotJuggleStatePerm(IntakeSubsystem Subsystem){
        
        super();

        subsystem = Subsystem;
        addRequirements(Subsystem);

    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        subsystem.IntakePiviotPID(45, 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopMotorPivot();
    }

    
    @Override
    public boolean isFinished() {
        return done;
    }
}

package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakePivotUpState extends Command {
        
IntakeSubsystem subsystem;

    double Position = 5;

    boolean done;

    public IntakePivotUpState(IntakeSubsystem Subsystem){
        
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
        subsystem.IntakePiviotPID(0, 1);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopMotorPivot();
    }

    /*
    @Override
    public boolean isFinished() {
        return done;
    }

    */
}

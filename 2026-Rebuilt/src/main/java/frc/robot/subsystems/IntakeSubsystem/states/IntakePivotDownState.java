package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakePivotDownState extends Command {
    
IntakeSubsystem subsystem;

    double Position = 5;

    boolean done;

    public IntakePivotDownState(IntakeSubsystem Subsystem){
        
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
    subsystem.RunMotorPivot(0.5);
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
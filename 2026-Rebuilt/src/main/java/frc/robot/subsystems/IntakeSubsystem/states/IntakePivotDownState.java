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
        /*if (subsystem.IntakePiviotPID(Position,1.0,0.1,0.5)) { //pos would be 24 climber
            done = true;
        }
        else{
            done = false;
        }
    */
    subsystem.RunMotorPivot(-0.1);
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
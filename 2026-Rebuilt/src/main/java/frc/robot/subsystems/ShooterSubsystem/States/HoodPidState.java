package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class HoodPidState extends Command {

    ShooterSubsystem subsystem;

    double Position;

    boolean done;

    public HoodPidState( ShooterSubsystem Subsystem){
        
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
        if (subsystem.HoodPID(Position,1.0,0.1,0.5)) {
            done = true;
        }
        else{
            done = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
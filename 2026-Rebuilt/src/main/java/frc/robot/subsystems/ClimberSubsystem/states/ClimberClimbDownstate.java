package frc.robot.subsystems.ClimberSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;

public class ClimberClimbDownstate extends Command{
    
ClimberSubsystem subsystem;

    double Position = 5;

    boolean done;

    public ClimberClimbDownstate(ClimberSubsystem Subsystem){
        
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
        if (subsystem.ClimbMotorPID(0,1,1,0.1)) { 
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


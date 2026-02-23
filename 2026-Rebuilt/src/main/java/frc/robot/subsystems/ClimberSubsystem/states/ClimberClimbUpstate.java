package frc.robot.subsystems.ClimberSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;

public class ClimberClimbUpstate extends Command {
    
ClimberSubsystem subsystem;

    double Position = 5;

    boolean done;

    public ClimberClimbUpstate(ClimberSubsystem Subsystem){
        
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
        /* 
        if (subsystem.ClimbMotorPID(24,1.0,0.1,0.1)) { 
        }
        else{
            done = false;
        }
        */
        subsystem.RunClimbMotor(0.75);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopClimbMotor();
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}

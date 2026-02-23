package frc.robot.subsystems.ClimberSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;

public class ClimberHoldUpstate extends Command{
    
    
    
ClimberSubsystem subsystem;

    double Position = 5;

    boolean done;

    public ClimberHoldUpstate(ClimberSubsystem Subsystem){
        
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
        if (subsystem.HoldMotorPID(24,1.0,0.1,0.1)) { // 1.213680 in radius for the gear relating to this motor
            done = true;
        }
        else{
            done = false;
        }
        */
        subsystem.RunHoldMotor(0.1);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
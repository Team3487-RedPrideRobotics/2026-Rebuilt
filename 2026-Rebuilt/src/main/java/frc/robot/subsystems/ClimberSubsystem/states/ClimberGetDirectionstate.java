package frc.robot.subsystems.ClimberSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;

import com.ctre.phoenix6.Utils;


public class ClimberGetDirectionstate extends Command {

ClimberSubsystem subsystem;

boolean done;
double startTime;
double current;

public ClimberGetDirectionstate(ClimberSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);

    }

    @Override
    public void initialize() {
        done = false;
        startTime=Utils.getCurrentTimeSeconds();
    }

    @Override
    public void execute() {
        subsystem.RunClimbMotor(1);

        if((subsystem.detectClimbMotorCurrent()-current)<=10){
            subsystem.SetDirection(-1);
            System.out.println("climber reversed!");
            done = true;
        }
        
        

        if(Utils.getCurrentTimeSeconds()-startTime > 1){
            subsystem.SetDirection(1);
            done = true;
        }

        current = subsystem.detectClimbMotorCurrent();
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopClimbMotor();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
  


}




  
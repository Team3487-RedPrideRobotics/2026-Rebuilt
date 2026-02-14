package frc.robot.subsystems.ClimberSubsystem.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.KickerSubsystem.kickerSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;


public class ClimberGetDirectionstate extends Command {

ClimberSubsystem subsystem;

boolean done;
double startTime;
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

        if(Utils.getCurrentTimeSeconds()-startTime > 1){
            done = true;
            subsystem.SetDirection(1);
        }
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




  
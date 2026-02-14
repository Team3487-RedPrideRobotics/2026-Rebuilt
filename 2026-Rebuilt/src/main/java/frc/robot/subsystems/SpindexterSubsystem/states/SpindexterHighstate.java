package frc.robot.subsystems.SpindexterSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexterSubsystem.SpindexterSubsytem;

public class SpindexterHighstate extends Command{
    
    SpindexterSubsytem subsystem;

    public SpindexterHighstate( SpindexterSubsytem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(1);
    }

    @Override
    public void end(boolean interrupted) {

    subsystem.StopMotors();

    }

    

}

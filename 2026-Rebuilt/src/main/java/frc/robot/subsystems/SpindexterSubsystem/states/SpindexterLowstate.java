package frc.robot.subsystems.SpindexterSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexterSubsystem.SpindexterSubsytem;

public class SpindexterLowstate extends Command{
    
    SpindexterSubsytem subsystem;


    public SpindexterLowstate( SpindexterSubsytem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(0);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopMotors();

    }

}
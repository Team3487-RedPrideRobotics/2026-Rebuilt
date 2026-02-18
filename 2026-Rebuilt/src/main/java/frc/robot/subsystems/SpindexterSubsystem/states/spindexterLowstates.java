package frc.robot.subsystems.SpindexterSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SpindexterSubsystem.SpindexterSubsytem;

public class SpindexterLowstates extends Command{
    
    SpindexterSubsytem subsystem;

    public SpindexterLowstates( SpindexterSubsytem Subsystem){
        
        subsystem = Subsystem;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(0.5);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopMotors();

    }

}
package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakeState extends Command{
    
    IntakeSubsystem subsystem;

    public IntakeState( IntakeSubsystem Subsystem){
        
        super();
        subsystem = Subsystem;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunIntake(0.75);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopIntake();

    }

    




}
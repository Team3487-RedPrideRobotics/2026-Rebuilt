package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

//run the outake but do NOT stop when canceled

public class OutakeStatePerm extends Command{
    
    IntakeSubsystem subsystem;

    public OutakeStatePerm( IntakeSubsystem Subsystem){
        
        super();
        subsystem = Subsystem;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunIntake(0.5);
    }

    @Override
    public void end(boolean interrupted) {

    }

    




}
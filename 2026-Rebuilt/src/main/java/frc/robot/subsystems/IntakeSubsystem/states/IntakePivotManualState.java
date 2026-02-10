package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class IntakePivotManualState extends Command{
    
    IntakeSubsystem subsystem;
    double PivotSpeed;

    public IntakePivotManualState(double speed, IntakeSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);
        PivotSpeed = speed;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotorPivot(PivotSpeed);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopIntake();

    }

    




}
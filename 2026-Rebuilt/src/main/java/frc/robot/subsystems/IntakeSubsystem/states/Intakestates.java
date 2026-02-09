package frc.robot.subsystems.IntakeSubsystem.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class Intakestates extends Command{
    
    IntakeSubsystem subsystem;

    double Speed;

    public Intakestates(double speed, IntakeSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);
        Speed = speed;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunIntake(Speed);
    }

    @Override
    public void end(boolean interrupted) {

        subsystem.StopMotorPivot();

    }

    




}
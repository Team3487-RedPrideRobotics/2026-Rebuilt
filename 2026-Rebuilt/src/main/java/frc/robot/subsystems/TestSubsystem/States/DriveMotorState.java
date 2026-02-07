package frc.robot.subsystems.TestSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TestSubsystem.*;

public class DriveMotorState extends Command {

    TestSubsystem subsystem;

    double Speed;

    public DriveMotorState(double speed, TestSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);
        Speed = speed;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.RunMotor(Speed);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
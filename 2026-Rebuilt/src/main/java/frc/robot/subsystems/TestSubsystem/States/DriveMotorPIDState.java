package frc.robot.subsystems.TestSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TestSubsystem.*;

public class DriveMotorPIDState extends Command {

    TestSubsystem subsystem;

    double Position;

    boolean done;

    public DriveMotorPIDState(double Turns, TestSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(Subsystem);
        Position = Turns;

    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (subsystem.SubsystemPID(Position,1.0,0.1,0.5)) {
            done = true;
        }
        else{
            done = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
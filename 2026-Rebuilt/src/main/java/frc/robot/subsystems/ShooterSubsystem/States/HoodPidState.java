package frc.robot.subsystems.ShooterSubsystem.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

public class HoodPidState extends Command {

    ShooterSubsystem subsystem;
    DoubleSupplier turretVelocity;
    double hoodAngle;
    

    public HoodPidState( ShooterSubsystem Subsystem, double customHoodAngle){
        
        addRequirements(Subsystem);
        
        subsystem = Subsystem;
        hoodAngle = customHoodAngle;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.HoodPID(hoodAngle,0.1,0.1,1);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
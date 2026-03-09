package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class Turret90cwPIDState extends Command {

    ShooterSubsystem subsystem;


    public Turret90cwPIDState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.TurretPIDFieldRelative(0);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopTurretMotor();
    }

}
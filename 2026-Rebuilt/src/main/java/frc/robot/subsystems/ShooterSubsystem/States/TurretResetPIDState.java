package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class TurretResetPIDState extends Command {

    ShooterSubsystem subsystem;


    public TurretResetPIDState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.TurretPIDRobotRelative(0);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopTurretMotor();
    }

}
package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.*;

public class TurretResetPIDState extends Command {

    ShooterSubsystem subsystem;

    boolean reset = false;

    public TurretResetPIDState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        System.out.println("trying to reset");
    }

    @Override
    public void execute() {
        reset = subsystem.TurretPIDRobotRelative(0);
    }

    @Override
    public boolean isFinished() {
        return reset;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopTurretMotor();
        System.out.println("reset successfully");
    }

}
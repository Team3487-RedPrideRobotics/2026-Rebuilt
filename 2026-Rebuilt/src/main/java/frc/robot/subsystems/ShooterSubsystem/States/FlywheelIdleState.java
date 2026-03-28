package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

public class FlywheelIdleState extends Command {

    ShooterSubsystem subsystem;

    public FlywheelIdleState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;

    }

    @Override
    public void execute() {
        subsystem.RunFlywheelMotor(3300/60);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopFlywheelMotors();
    }

}
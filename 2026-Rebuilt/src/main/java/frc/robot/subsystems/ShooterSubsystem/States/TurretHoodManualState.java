package frc.robot.subsystems.ShooterSubsystem.States;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.SubsystemConstants;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

public class TurretHoodManualState extends Command {

    ShooterSubsystem subsystem;
    DoubleSupplier turretVelocity;
    DoubleSupplier hoodSpeed;
    

    public TurretHoodManualState( ShooterSubsystem Subsystem, DoubleSupplier TurretVelocity, DoubleSupplier hoodSpeed){
        
        addRequirements(Subsystem);
        
        subsystem = Subsystem;
        this.turretVelocity = TurretVelocity;
        this.hoodSpeed = hoodSpeed;

    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        subsystem.TurretPIDRobotRelative(subsystem.getTurretAngle()+SubsystemConstants.TurretRotationSpeed*turretVelocity.getAsDouble());
        subsystem.RunHoodMotor(hoodSpeed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
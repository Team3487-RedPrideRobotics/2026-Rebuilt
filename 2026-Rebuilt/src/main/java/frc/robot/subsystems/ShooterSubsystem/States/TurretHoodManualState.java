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
        if(turretVelocity.getAsDouble() != 0 ){
        subsystem.RunTurretMotor(turretVelocity.getAsDouble()*SubsystemConstants.TurretRotationSpeed);
        }
        else{
            subsystem.StopTurretMotor();
        }
        //subsystem.setAngle(subsystem.DegreesAngleClamp(subsystem.getTurretAngle()+turretVelocity.getAsDouble()*SubsystemConstants.TurretRotationSpeed));
        if(hoodSpeed.getAsDouble() != 0){
        subsystem.RunHoodMotor(hoodSpeed.getAsDouble()*0.1);}
        else{
            subsystem.StopHoodMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }

}
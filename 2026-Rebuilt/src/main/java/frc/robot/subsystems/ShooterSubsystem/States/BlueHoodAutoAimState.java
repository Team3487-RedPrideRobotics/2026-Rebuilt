package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.LimelightConstants;
import frc.robot.subsystems.ShooterSubsystem.*;

public class BlueHoodAutoAimState extends Command {

    ShooterSubsystem subsystem;

    double Position;

    boolean done;

    public BlueHoodAutoAimState( ShooterSubsystem Subsystem){
        
        super();
        subsystem = Subsystem;
        addRequirements(Subsystem);
        

    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (subsystem.FullTurretAutoAim(LimelightConstants.BlueHubPose2d, 0.1)) {
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
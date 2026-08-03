package frc.robot.subsystems.ShooterSubsystem.States;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.VisionConstants;
import frc.robot.subsystems.ShooterSubsystem.*;

public class AutoHubAimState extends Command {

    ShooterSubsystem subsystem;

    public AutoHubAimState( ShooterSubsystem Subsystem){
        
        subsystem = Subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void execute() {
        System.out.println("Aiming");
        subsystem.FullTurretAutoAim(DriverStation.getAlliance().get() == Alliance.Red ?VisionConstants.RedHubPose2d : VisionConstants.BlueHubPose2d);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.StopHoodMotor();
        subsystem.StopTurretMotor();
    }

}
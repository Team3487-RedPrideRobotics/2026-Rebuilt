// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final CommandSwerveDrivetrain m_Drivetrain;

    private final PoseEstimatorSubsystem m_PoseEstimator;

    private final Alert NoFmsAlliance;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_Drivetrain = m_robotContainer.m_drivetrain;
        m_PoseEstimator = m_robotContainer.m_PoseEstimator;
        //Allows Operator to know if the fms is disconected
        NoFmsAlliance = new Alert("The FMS is not sending Alliance!", AlertType.kError);

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {
        m_Drivetrain.seedFieldCentric();
        m_PoseEstimator.setThermalManagement(true);
    }

    @Override
    public void disabledPeriodic() {
        if(DriverStation.isDSAttached()){
            NoFmsAlliance.set(DriverStation.getAlliance().isEmpty() ? true: false);
        }
    }

    @Override
    public void disabledExit() {
        m_PoseEstimator.setThermalManagement(false);
    }

    @Override
    public void autonomousInit() {
        m_Drivetrain.seedFieldCentric();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {
        CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}

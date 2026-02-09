// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.Swerve.SwerveCommands.LimelightChassisAimState;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.RobotCentric RobotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry m_logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);

    

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public final PoseEstimatorSubsystem  m_PoseEstimator = new PoseEstimatorSubsystem(m_drivetrain);

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //when Y on the driver controller is pressed, toggle Robot Centric Driving.
        driverController.y().toggleOnTrue(m_drivetrain.applyRequest(() -> 
            RobotCentricDrive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate)));
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(driverController.getLeftY(), driverController.getLeftX()))
        ));
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
        
        driverController.rightBumper().whileTrue(new LimelightChassisAimState(m_drivetrain, RobotCentricDrive,new Pose2d(0.0,-1.0,Rotation2d.kZero)));
        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auto
        return autoChooser.getSelected();
    }
}

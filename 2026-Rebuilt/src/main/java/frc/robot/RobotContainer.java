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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.SubsystemConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.states.ClimberClimbDownstate;
import frc.robot.subsystems.ClimberSubsystem.states.ClimberClimbUpstate;
import frc.robot.subsystems.ClimberSubsystem.states.ClimberGetDirectionstate;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.states.IntakePivotDownState;
import frc.robot.subsystems.IntakeSubsystem.states.IntakePivotUpState;
import frc.robot.subsystems.IntakeSubsystem.states.IntakeState;
import frc.robot.subsystems.IntakeSubsystem.states.OutakeState;
import frc.robot.subsystems.IntakeSubsystem.states.OutakeStatePerm;
import frc.robot.subsystems.KickerSubsystem.kickerSubsystem;
import frc.robot.subsystems.KickerSubsystem.states.KickerFeedState;
import frc.robot.subsystems.KickerSubsystem.states.KickerReverseState;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.States.AutoHubAimStateOff;
import frc.robot.subsystems.ShooterSubsystem.States.AutoHubAimStatePerm;
import frc.robot.subsystems.ShooterSubsystem.States.BlueHoodAutoAimState;
import frc.robot.subsystems.ShooterSubsystem.States.FlywheelIdleState;
import frc.robot.subsystems.ShooterSubsystem.States.FlywheelIdleStatePerm;
import frc.robot.subsystems.ShooterSubsystem.States.HoodDownState;
import frc.robot.subsystems.ShooterSubsystem.States.TurretSlowblowPIDState;
import frc.robot.subsystems.ShooterSubsystem.States.TurretHoodManualState;
import frc.robot.subsystems.ShooterSubsystem.States.TurretResetPIDState;
import frc.robot.subsystems.SpindexterSubsystem.SpindexterSubsytem;
import frc.robot.subsystems.SpindexterSubsystem.states.SpindexterHighstate;
import frc.robot.subsystems.SpindexterSubsystem.states.SpindexterLowstate;
import frc.robot.subsystems.SpindexterSubsystem.states.SpindexterReversestate;
import frc.robot.subsystems.Swerve.CommandSwerveDrivetrain;

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
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt(); //unused but may come back for

    private final Telemetry m_logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverController = new CommandXboxController(1);
    private final CommandXboxController operatorController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public final PoseEstimatorSubsystem  m_PoseEstimator = new PoseEstimatorSubsystem(m_drivetrain);

    
        //instance other robot subsystems
        public final kickerSubsystem    m_kicker       = new kickerSubsystem();
        public final SpindexterSubsytem m_Spindexter   = new SpindexterSubsytem();
        public final IntakeSubsystem    m_Intake       = new IntakeSubsystem();
        public final ShooterSubsystem   m_Shooter      = new ShooterSubsystem(m_PoseEstimator,this);
        public final ClimberSubsystem   m_Climber      = new ClimberSubsystem();
    
        //checking which aliance that the robot is on
        public boolean IsRed; 
    
        private final SendableChooser<Command> autoChooser;
    
        private Alliance m_alliance;
    
        public CommandXboxController getDriveController(){
            return this.driverController;
        }
        
        public Alliance getAlliance(){
            DriverStation.getAlliance().ifPresent((DriverStation.Alliance myAlliance) -> {
                m_alliance = myAlliance;
                if(m_alliance == Alliance.Red){
                    IsRed = true;
                }
                else{IsRed = false;}
            });
            return m_alliance;
        } 
    
        public final RobotContainer getInstance(){
            return this;
        }

    public RobotContainer() {

        getAlliance();

        buildNamedCommands();

        configureBindings();

        DriverStation.silenceJoystickConnectionWarning(true);

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {

        //CONFIGIURE DRIVER CONTROLS

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

        //when Y on the driver controller is pressed, toggle Robot Centric Driving (toggle)
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
        //brake the robot's movement
        driverController.a().whileTrue(m_drivetrain.applyRequest(() -> brake));

        /* 
        driverController.b().whileTrue(m_drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(driverController.getLeftY(), driverController.getLeftX()))
        ));*/
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on b press
        driverController.b().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        // Robot intake
        driverController.rightTrigger().whileTrue(new IntakeState(m_Intake)); // while the LT button is held it will intake fuel
        driverController.leftTrigger().whileTrue(new OutakeState(m_Intake));  // while the LB button is held it will outake fuel
        driverController.leftBumper().whileTrue(new IntakePivotUpState(m_Intake));  // when RT button is pressed retract the intake 
        driverController.rightBumper().whileTrue(new IntakePivotDownState(m_Intake));  // when RB button is pressed deploy the intake
        
        //Non Competition viable in current state
        //driverController.rightBumper().whileTrue(new LimelightChassisAimState(m_drivetrain, RobotCentricDrive,new Pose2d(0.0,-1.0,Rotation2d.kZero)));




        //OPERATOR CONTROLS

        //Automatically set th turret to follow manual control and for the spindexter to spin
        m_Spindexter.setDefaultCommand(
            new ParallelCommandGroup(
                new SpindexterLowstate(m_Spindexter),
                new TurretHoodManualState(m_Shooter, () -> MathUtil.applyDeadband(operatorController.getLeftX(),SubsystemConstants.OperatorConstants.leftXdeadBand)
                                                   , () -> MathUtil.applyDeadband(operatorController.getLeftY(),SubsystemConstants.OperatorConstants.leftYdeadBand))
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)));

        //auto aims the Turret, Hood, and Flywheel to the hub; aims the turret to snowblow; resets to front
        operatorController.leftTrigger().whileTrue(new BlueHoodAutoAimState(m_Shooter,this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        operatorController.leftBumper().whileTrue(new TurretSlowblowPIDState(m_Shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        operatorController.y().whileTrue(new TurretResetPIDState(m_Shooter));

        //activate the kicker and spindexter to feed fuel into the shooter to effectively shoot
        operatorController.rightTrigger(0.5).whileTrue(
            new ParallelCommandGroup(
                    new KickerFeedState(m_kicker),
                    new SpindexterHighstate(m_Spindexter)
                ));

        
        operatorController.b().whileTrue(new HoodDownState(m_Shooter));
        
        //spinning the flywheel up to idle speed(3000rpm) on Y (toggle)
        operatorController.rightBumper().whileTrue(new FlywheelIdleState(m_Shooter
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //switch the direction of the spindexter
        operatorController.x().whileTrue(new ParallelCommandGroup(new SpindexterReversestate(m_Spindexter),
                                                                            new KickerReverseState(m_kicker))
                                                                            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        // Moving the climber arm up/down
        operatorController.povUp().whileTrue(new ClimberClimbUpstate(m_Climber));
        operatorController.povDown().whileTrue(new ClimberClimbDownstate(m_Climber));

        //Start up swerve telemetry logging
        m_drivetrain.registerTelemetry(m_logger::telemeterize);
    }  

    public void buildNamedCommands(){
        //NamedCommands.registerCommand("Flywheel Spinup", new FlywheelIdleStatePerm(m_Shooter).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        //NamedCommands.registerCommand("Climber Diretion Test", new ClimberGetDirectionstate(m_Climber));
        //Shooter:
        //NamedCommands.registerCommand("Turret Auto Aim",new BlueHoodAutoAimState(m_Shooter,this));
        //NamedCommands.registerCommand("Stop Turret Auto Aim",new AutoHubAimStateOff(m_Shooter,this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        //new EventTrigger("Turret Auto Aim").whileTrue(new BlueHoodAutoAimState(m_Shooter,this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        //NamedCommands.registerCommand("Turret Shoot",new ParallelCommandGroup(
        //                                                                        new KickerFeedState(m_kicker),
        //                                                                        new SpindexterHighstate(m_Spindexter)
        //                                                                          ));
        //Climber:
        //NamedCommands.registerCommand("Climber Arm Extend", new ClimberClimbUpstate(m_Climber).withTimeout(2));
        //Intake:
        //NamedCommands.registerCommand("Intake Extend", new IntakePivotDownState(m_Intake).withTimeout(1));
        //NamedCommands.registerCommand("Intake Retract", new IntakePivotUpState(m_Intake).withTimeout(1));
        //NamedCommands.registerCommand("Intake", new IntakeState(m_Intake).withTimeout(5));
        //NamedCommands.registerCommand("OutakePerm", new OutakeStatePerm(m_Intake));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}

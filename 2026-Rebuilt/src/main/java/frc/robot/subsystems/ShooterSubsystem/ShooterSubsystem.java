


package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.LimelightConstants;
import frc.robot.generated.SubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem.States.HoodDownState;
import frc.robot.subsystems.ShooterSubsystem.States.HoodPidState;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;
    private TalonFX m_TurretMotor;
    private TalonFX m_HoodMotor;

    private PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
    private RobotContainer m_RobotContainer;

    private VelocityDutyCycle m_FlywheelMotorRequest;
    private DutyCycleOut m_TurretMotorRequest;
    private DutyCycleOut m_HoodMotorRequest;

    private Slot0Configs m_FlywheelMotorSlotConfigs;

    boolean TurretAimed = false;

    Alert TurretRingOverrun = new Alert("Turret ring overrun!", AlertType.kWarning);

    double CustomShooterSpeed = 0.0;

    double CustomHoodAngle = 0.0;

    public ShooterSubsystem(PoseEstimatorSubsystem Goku, RobotContainer Vegeta) {

        m_PoseEstimatorSubsystem = Goku;
        m_RobotContainer = Vegeta;

        m_FlywheelMotor = new TalonFX(SubsystemConstants.ShooterFlywheelKrakenCANID);
        m_TurretMotor = new TalonFX(SubsystemConstants.ShooterTurretKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
        m_HoodMotor = new TalonFX(SubsystemConstants.ShooterHoodKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

        m_FlywheelMotorRequest = new VelocityDutyCycle(0.0);
        m_TurretMotorRequest = new DutyCycleOut(0.0);
        m_HoodMotorRequest = new DutyCycleOut(0);

        m_FlywheelMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.ShooterFlywheelKrakenInverted));
        m_TurretMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.ShooterTurretKrakenInverted));

        m_FlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        m_TurretMotor.setNeutralMode(NeutralModeValue.Brake);
        m_HoodMotor.setNeutralMode(NeutralModeValue.Brake);

        m_FlywheelMotorSlotConfigs = new Slot0Configs();
        m_FlywheelMotorSlotConfigs.kS = 0.1;
        m_FlywheelMotorSlotConfigs.kA = 1;
        m_FlywheelMotorSlotConfigs.kP = 1;

        m_FlywheelMotor.getConfigurator().apply(m_FlywheelMotorSlotConfigs);
        SmartDashboard.putBoolean("TurretAimed", TurretAimed);
        SmartDashboard.putNumber("Custom Shooter Speed", CustomShooterSpeed);
        SmartDashboard.putNumber("Custom Hood Angle", CustomHoodAngle);
    }

    //takes a value above 360 degrees and wraps it around to 0
    public double DegreesAngleClamp(double angle){
        return((angle/360-Math.floor(angle/360))*360);
    }

    //returns an angle of degrees from the input o
    public double TurretTurnsToDeg(double turns){
        return(((turns*SubsystemConstants.ShooterTurretGearRatio)-Math.floor(turns*SubsystemConstants.ShooterTurretGearRatio))/360);
    }

    // returns a number of relative turns of the turret motor
    public double DegToTurretTurns(double degrees, double gearRatio){
        return((degrees*1/gearRatio)/360);
    }

    //Flywheel Control
    public void RunFlywheelMotor(double speed) {
        if(CustomShooterSpeed != 0){
            m_FlywheelMotorRequest.withVelocity(CustomShooterSpeed);
        }
        else{
        m_FlywheelMotorRequest.withVelocity(speed);
        }
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    public void StopFlywheelMotors() {
        m_FlywheelMotorRequest.withVelocity(0);
        m_FlywheelMotor.stopMotor();
    }

    //returns Flywheel RPM
    public double getFlywheelSpeed(){
        return(m_FlywheelMotor.getVelocity().getValueAsDouble()*60);
    }

    //This code IS currently operational due to the current robot design
    //Hood Control
    public void RunHoodMotor(double speed) {

        m_HoodMotorRequest.Output = m_HoodMotor.getPosition().getValueAsDouble() > SubsystemConstants.ShooterHoodHardLimitTop ? -speed : speed;
        m_HoodMotorRequest.Output = m_HoodMotor.getPosition().getValueAsDouble() < SubsystemConstants.ShooterHoodHardLimitBottom ? -speed : speed; 
        m_HoodMotor.setControl(m_HoodMotorRequest);
    }

    public void StopHoodMotor() {
        m_HoodMotorRequest.Output = 0;
        m_HoodMotor.stopMotor();
    }

    //Goal in turns, Limit in max speed, kP as P value, threshold as in tolerance
    public boolean HoodPID(double goalValue, double limit, double kP, double threshold) {
        double delta = Math.abs(goalValue) - Math.abs(m_HoodMotor.getPosition().getValueAsDouble());
        if (Math.abs(delta) >= threshold) {
            var speed = -delta * kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunHoodMotor(speed);
            return false;
        } else {
            StopHoodMotor();
            return true;
        }

    }

    public double getHoodTurns(){
        return(m_HoodMotor.getPosition().getValueAsDouble());
    }
    

    //Turret Control
    public void RunTurretMotor(double speed) {
        
        m_TurretMotorRequest.withOutput(m_TurretMotor.getPosition().getValueAsDouble() > SubsystemConstants.ShooterHoodHardLimitTop ? -speed : speed);
        m_TurretMotorRequest.withOutput(m_TurretMotor.getPosition().getValueAsDouble() < SubsystemConstants.ShooterHoodHardLimitBottom ? -speed : speed); 
        
        m_TurretMotor.setControl(m_TurretMotorRequest);
    }

    public void StopTurretMotor() {
        m_TurretMotorRequest.withOutput(0);
        m_TurretMotor.stopMotor();
    }

    public boolean TurretPID(double goalValue, double limit, double kP, double threshold) {
        if(goalValue >= SubsystemConstants.ShooterTurretHardLimitTop || goalValue <= SubsystemConstants.ShooterTurretHardLimitBottom){
        TurretRingOverrun.set(false);
        double delta = Math.abs(goalValue) - Math.abs(m_TurretMotor.getPosition().getValueAsDouble());
        if (Math.abs(delta) >= threshold) {
            var speed = -delta * kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunTurretMotor(speed);
            return false;
        } else {
            StopTurretMotor();
            return true;
        }
    }
    else{
        //System.err.println("Turret Ring Overrun!");
        TurretRingOverrun.set(true);

        return(false);
    }}

    public double getTurretAngle(){
        return(TurretTurnsToDeg(m_TurretMotor.getPosition().getValueAsDouble()));
    }

    public boolean TurretPIDAngle(double angle){
    return(TurretPID(DegToTurretTurns(DegreesAngleClamp(angle),SubsystemConstants.ShooterTurretGearRatio), 1, 0.1, 0.02));
    }

    //AimPose: the pose to aim at, tolearance: how close it finds 'acceptable' in deg
    public boolean FullTurretAutoAim(Pose2d AimPose,double tolerance){
        Pose2d robotPose = m_PoseEstimatorSubsystem.getRobotPose2d();
        Translation2d chassisSpeed = new Translation2d(m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vxMetersPerSecond
                                                      ,m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vyMetersPerSecond)
                                                      .rotateBy(robotPose.getRotation());
        double distanceToHub;
        double desiredHoodAngle;
        double desiredRPM;
        Rotation2d desiredTurretAngle;
            desiredTurretAngle = new Rotation2d(Math.atan2(AimPose.relativeTo(robotPose).getY()+chassisSpeed.getY(),AimPose.relativeTo(robotPose).getX()+chassisSpeed.getX()));
            distanceToHub = getDistanceToHub(robotPose);
            desiredHoodAngle = LimelightConstants.TurretHoodInterpolatorDEG.get(distanceToHub);
            desiredRPM = LimelightConstants.TurretFlywheelInterpolatorRPM.get(distanceToHub);
        if(Math.abs(desiredHoodAngle-getTurretAngle()) < tolerance /*&& Math.abs(desiredHoodAngle-getHoodTurns()*360)<tolerance*/){
            RunFlywheelMotor(2500.0/60);
            TurretAimed = true;
            return true;
        }
        else{
            TurretPIDAngle(desiredTurretAngle.getDegrees());
            RunFlywheelMotor(desiredRPM/60);
            HoodPID(desiredHoodAngle, 0.1, 0.1, tolerance);
            TurretAimed = false;
            return false;
        }

    }

    public double getDistanceToHub(Pose2d hubPose2d){
        Pose2d robotPose = m_PoseEstimatorSubsystem.getRobotPose2d();
        Translation2d chassisSpeed = new Translation2d(m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vxMetersPerSecond
                                                      ,m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vyMetersPerSecond)
                                                      .rotateBy(robotPose.getRotation());
        double distanceToHub;
        distanceToHub = (hubPose2d.relativeTo(robotPose).getTranslation().plus(chassisSpeed)).getNorm();
        return distanceToHub;
    }

    @Override
    public void periodic() {
        //m_RobotContainer.getDriveController().setRumble(RumbleType.kBothRumble, (getDistanceToHub(m_RobotContainer.IsRed ? LimelightConstants.RedHubPose2d:LimelightConstants.BlueHubPose2d)-2.1336/2.7432));
        //System.out.println(m_TurretMotor.getPosition().getValueAsDouble());
        m_PoseEstimatorSubsystem.getField2d().getObject("estimated shot").setPose(new Pose2d(new Translation2d(LimelightConstants.InverseTurretFlywheelInterpolatorRPM.get(getFlywheelSpeed()),0).rotateBy(new Rotation2d(getTurretAngle()).plus(m_PoseEstimatorSubsystem.getRobotPose2d().getRotation())),new Rotation2d()));
        CustomShooterSpeed = SmartDashboard.getNumber("Custom Shooter Speed", 0);
        CustomHoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0);
            if(CustomHoodAngle != 0){
            CommandScheduler.getInstance().schedule(new HoodPidState(this,CustomHoodAngle).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
            }
    }
}
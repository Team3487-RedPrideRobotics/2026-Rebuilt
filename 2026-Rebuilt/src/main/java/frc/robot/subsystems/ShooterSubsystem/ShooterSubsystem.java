
package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.LimelightConstants;
import frc.robot.generated.SubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;
    private TalonFX m_TurretMotor;
    private TalonFX m_HoodMotor;

    private PoseEstimatorSubsystem m_PoseEstimatorSubsystem;

    private VelocityDutyCycle m_FlywheelMotorRequest;
    private DutyCycleOut m_TurretMotorRequest;
    private DutyCycleOut m_HoodMotorRequest;

    Alert TurretRingOverrun = new Alert("Turret ring overrun!", AlertType.kWarning);

    public ShooterSubsystem(PoseEstimatorSubsystem Goku) {

        m_PoseEstimatorSubsystem = Goku;

        m_FlywheelMotor = new TalonFX(SubsystemConstants.ShooterFlywheelKrakenCANID);
        m_TurretMotor = new TalonFX(SubsystemConstants.ShooterTurretKrakenCANID);
        m_HoodMotor = new TalonFX(SubsystemConstants.ShooterHoodKrakenCANID);

        m_FlywheelMotorRequest = new VelocityDutyCycle(0.0);
        m_TurretMotorRequest = new DutyCycleOut(0.0);
        m_HoodMotorRequest = new DutyCycleOut(0);

        m_FlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        m_TurretMotor.setNeutralMode(NeutralModeValue.Brake);
        m_HoodMotor.setNeutralMode(NeutralModeValue.Brake);

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
        m_FlywheelMotorRequest.Velocity = speed;
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    public void StopFlywheelMotors() {
        m_FlywheelMotorRequest.Velocity = 0;
        m_FlywheelMotor.stopMotor();
    }

    //returns Flywheel RPM
    public double getFlywheelSpeed(){
        return(m_FlywheelMotor.getVelocity().getValueAsDouble()*60);
    }

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

        m_TurretMotorRequest.Output = m_TurretMotor.getPosition().getValueAsDouble() > SubsystemConstants.ShooterHoodHardLimitTop ? -speed : speed;
        m_TurretMotorRequest.Output = m_TurretMotor.getPosition().getValueAsDouble() < SubsystemConstants.ShooterHoodHardLimitBottom ? -speed : speed; 
        m_TurretMotor.setControl(m_TurretMotorRequest);
    }

    public void StopTurretMotor() {
        m_TurretMotorRequest.Output = 0;
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
        System.err.println("Turret Ring Overrun!");
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
            desiredTurretAngle = Rotation2d.fromDegrees(Math.atan2(AimPose.relativeTo(robotPose).getY()+chassisSpeed.getY(),AimPose.relativeTo(robotPose).getX()+chassisSpeed.getX()));
            distanceToHub = getDistanceToHub(robotPose);
            desiredHoodAngle = LimelightConstants.TurretHoodInterpolatorDEG.get(distanceToHub);
            desiredRPM = LimelightConstants.TurretFlywheelInterpolatorRPM.get(distanceToHub);
        if(Math.abs(desiredHoodAngle-getTurretAngle()) < tolerance && Math.abs(desiredHoodAngle-getHoodTurns()*360)<tolerance){
            RunFlywheelMotor(2500.0);
            return true;
        }
        else{
            TurretPIDAngle(desiredTurretAngle.getDegrees());
            RunFlywheelMotor(desiredRPM);
            HoodPID(desiredHoodAngle, 0.1, 0.1, tolerance);
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
}
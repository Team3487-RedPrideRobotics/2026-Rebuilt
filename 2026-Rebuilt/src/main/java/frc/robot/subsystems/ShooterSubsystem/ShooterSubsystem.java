


package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.LimelightConstants;
import frc.robot.generated.SubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;
    private TalonFX m_TurretMotor;
    private TalonFX m_HoodMotor;

    private PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
    private RobotContainer m_RobotContainer;

    private VelocityDutyCycle m_FlywheelMotorRequest;
    private DutyCycleOut m_TurretMotorRequest;
    private PositionDutyCycle m_TurretPositionRequest;
    private DutyCycleOut m_HoodMotorRequest;
    

    private Slot0Configs m_FlywheelMotorSlotConfigs;

    boolean TurretAimed = false;

    Alert TurretRingOverrun = new Alert("Turret ring overrun!", AlertType.kWarning);

    double CustomShooterSpeed = 0.0;

    double CustomHoodAngle = 0.0;

    double FlywheelRPM = 0;

    double TurretAngle = 0;

    public ShooterSubsystem(PoseEstimatorSubsystem Goku, RobotContainer Vegeta) {

        m_PoseEstimatorSubsystem = Goku;
        m_RobotContainer = Vegeta;

        m_FlywheelMotor = new TalonFX(SubsystemConstants.ShooterFlywheelKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
        m_TurretMotor = new TalonFX(SubsystemConstants.ShooterTurretKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
        m_HoodMotor = new TalonFX(SubsystemConstants.ShooterHoodKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

        m_FlywheelMotorRequest = new VelocityDutyCycle(0.0);
        m_TurretMotorRequest = new DutyCycleOut(0.0);
        m_TurretPositionRequest = new PositionDutyCycle(0.0);
        m_HoodMotorRequest = new DutyCycleOut(0);

        m_FlywheelMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.ShooterFlywheelKrakenInverted));
        m_TurretMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.ShooterTurretKrakenInverted));

        m_FlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration m_TurretConfig = new TalonFXConfiguration();
        TalonFXConfiguration m_FlywheelConfig = new TalonFXConfiguration();
        TalonFXConfiguration m_HoodConfig = new TalonFXConfiguration();

        //Flywheel configs
        m_FlywheelMotorSlotConfigs = new Slot0Configs();
        m_FlywheelMotorSlotConfigs.kS = 0.1;
        m_FlywheelMotorSlotConfigs.kA = 1;
        m_FlywheelMotorSlotConfigs.kP = 1;
        m_FlywheelConfig.Slot0 = m_FlywheelMotorSlotConfigs;
        m_FlywheelConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_FlywheelConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        //turret configs
        Slot0Configs slot0 = m_TurretConfig.Slot0;
        slot0.kP = 1;//5
        slot0.kI = 0;//0.6
        slot0.kD = 0.1;//0.3

        SoftwareLimitSwitchConfigs softLimitsTurret = m_TurretConfig.SoftwareLimitSwitch;
        softLimitsTurret.ForwardSoftLimitThreshold = SubsystemConstants.ShooterTurretHardLimitTop;
        softLimitsTurret.ForwardSoftLimitEnable = true;
        softLimitsTurret.ReverseSoftLimitThreshold = SubsystemConstants.ShooterTurretHardLimitBottom;
        softLimitsTurret.ReverseSoftLimitEnable = true;

        m_TurretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_TurretConfig.Feedback.SensorToMechanismRatio = SubsystemConstants.ShooterTurretGearRatio;

        //hood configs

        SoftwareLimitSwitchConfigs softLimitsHood = m_HoodConfig.SoftwareLimitSwitch;
        softLimitsHood.ForwardSoftLimitThreshold = SubsystemConstants.ShooterHoodHardLimitTop;
        softLimitsHood.ForwardSoftLimitEnable = true;
        softLimitsHood.ReverseSoftLimitThreshold = SubsystemConstants.ShooterHoodHardLimitBottom;
        softLimitsHood.ReverseSoftLimitEnable = true;

        m_HoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //Apply configs
        m_TurretMotor.getConfigurator().apply(m_TurretConfig);
        m_FlywheelMotor.getConfigurator().apply(m_FlywheelConfig);
        m_HoodMotor.getConfigurator().apply(m_HoodConfig);

        m_TurretMotor.setPosition(0);
        m_HoodMotor.setPosition(0);

        SmartDashboard.putBoolean("TurretAimed", TurretAimed);
        SmartDashboard.putNumber("Custom Hood Angle", CustomHoodAngle);
        SmartDashboard.putNumber("Shooter RPM", FlywheelRPM);
        SmartDashboard.putNumber("Shooter Angle", TurretAngle);
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
        m_FlywheelMotorRequest.withVelocity(speed);
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

    //Hood Control
    public void RunHoodMotor(double speed) {
        m_HoodMotorRequest.withOutput(speed);
        m_HoodMotor.setControl(m_HoodMotorRequest);
    }

    public void StopHoodMotor() {
        m_HoodMotorRequest.Output = 0;
        m_HoodMotor.stopMotor();
    }

    //Goal in Deg, Limit in max speed, kP as P value(influencing speed), threshold as in tolerance
    public boolean HoodPID(double goalValue, double limit, double kP, double threshold) {
        double delta = Math.abs(goalValue) - Math.abs(getHoodAngle().getDegrees());
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

    public Rotation2d getHoodAngle(){
        Rotation2d hoodAngle = new Rotation2d();
        hoodAngle = Rotation2d.fromDegrees(getHoodTurns()*SubsystemConstants.ShooterHoodGearRatio*360+SubsystemConstants.ShooterHoodLowestAngle);
        return hoodAngle;
    }
    
    
    //Turret Control
    public void RunTurretMotor(double speed) {
        m_TurretMotorRequest.withOutput(speed);        
        m_TurretMotor.setControl(m_TurretMotorRequest);
    }

    public void StopTurretMotor() {
        m_TurretMotorRequest.withOutput(0);
        m_TurretMotor.stopMotor();
    }

    public void setAngle(double angleDegrees) {
    setAngle(angleDegrees, 0);
    }

    public void setAngle(double angleDegrees, double acceleration) {
    // Convert degrees to rotations
    double angleRadians = Units.degreesToRadians(angleDegrees);
    double positionRotations = angleRadians / (2.0 * Math.PI);
    
    m_TurretPositionRequest.withPosition(positionRotations);
    }

    public boolean TurretPIDRobotRelative(double angle){
        boolean done;
        double delta;
        delta = Math.abs(angle-getTurretAngle());
        done = true;
        if(delta > 0.1){
        setAngle(angle+SubsystemConstants.ShooterCenteredRotation
                        ,SubsystemConstants.TurretRotationSpeed);
        done = false;
        }
        return done;
    }

    public boolean TurretPIDFieldRelative(double angle){
        boolean done;
        done = TurretPIDRobotRelative(m_PoseEstimatorSubsystem.getRobotPose2d().getRotation().getDegrees()-angle);
        return done;
    }

    public double getTurretAngle(){
        double angle;
        angle = m_TurretMotor.getPosition().getValueAsDouble();
        angle = TurretTurnsToDeg(angle*360);
        return angle;
    }

    public double getTurretAngleRobotRelative(){
        double angle;
        angle = getTurretAngle();
        angle = angle-SubsystemConstants.ShooterCenteredRotation;
        return angle;
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
            TurretAimed = true;
            return true;
        }
        else{
            TurretPIDFieldRelative(desiredTurretAngle.getDegrees());
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
        CustomShooterSpeed = SmartDashboard.getNumber("Custom Shooter Speed", 0);
        FlywheelRPM = getFlywheelSpeed();
        TurretAngle = m_TurretMotor.getPosition().getValueAsDouble();
        SmartDashboard.updateValues();
    }
}
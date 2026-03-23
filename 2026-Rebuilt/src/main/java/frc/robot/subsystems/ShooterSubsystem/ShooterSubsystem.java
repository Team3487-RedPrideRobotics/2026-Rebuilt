package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.generated.LimelightConstants;
import frc.robot.generated.SubsystemConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

//TODO: Remove Turret Angle Adjust (its not used anyway)

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;
    public  TalonFX m_TurretMotor;
    private TalonFX m_HoodMotor;
    private final SingleJointedArmSim m_TurretSim;

    private PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
    private RobotContainer         m_RobotContainer;

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
    boolean constantAutoAim = false;
    boolean constantSnowblow = false;
    boolean constantGoalRedAlliance = false;

    private final StatusSignal<Angle> TurretAngle;

    public ShooterSubsystem(PoseEstimatorSubsystem Goku, RobotContainer myLittleRobotContainer) {

        m_PoseEstimatorSubsystem = Goku;
        m_RobotContainer = myLittleRobotContainer;

        m_TurretSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1), 
        10,
        0.05, // Arm moment of inertia
        0, // Arm length (m)
        Units.degreesToRadians(0), // Min angle of the motor (deg)
        Units.degreesToRadians( 360), // Max angle of the motor(deg)
        false, // Simulate gravity NO
        Units.degreesToRadians(0) // Starting position (rad)
        );

        m_FlywheelMotor = new TalonFX(SubsystemConstants.ShooterFlywheelKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
        m_TurretMotor = new TalonFX(SubsystemConstants.ShooterTurretKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
        m_HoodMotor = new TalonFX(SubsystemConstants.ShooterHoodKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

        TurretAngle = m_TurretMotor.getPosition();

        m_FlywheelMotorRequest = new VelocityDutyCycle(0.0).withIgnoreHardwareLimits(true);
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
        m_FlywheelMotorSlotConfigs.kD = 0.2;
        m_FlywheelConfig.Slot0 = m_FlywheelMotorSlotConfigs;
        m_FlywheelConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        m_FlywheelConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        m_FlywheelConfig.HardwareLimitSwitch.withForwardLimitEnable(false);

        //turret configs
        Slot0Configs slot0 = m_TurretConfig.Slot0;
        slot0.kP = 0.25;//5
        slot0.kI = 0;//0.6
        slot0.kD = 0.05;//0.3

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
    double positionRotations = DegreesAngleClamp(angleDegrees)/360*10;
    if(positionRotations<SubsystemConstants.ShooterTurretHardLimitTop
       &&positionRotations>SubsystemConstants.ShooterTurretHardLimitBottom){
        m_TurretPositionRequest.withPosition(positionRotations);
        m_TurretMotor.setControl(m_TurretPositionRequest);
        TurretRingOverrun.set(false);
       }
    else{
        m_TurretMotorRequest.withOutput(0);//stop motors
        m_TurretMotor.setControl(m_TurretMotorRequest);
        m_TurretMotor.stopMotor(); //set rumble for driver
        TurretRingOverrun.set(true); //set Overun to true
    }
    }

    public boolean TurretPIDRobotRelative(double angle){
        boolean done;
        double delta;
        double TurretRobotRelativeAngle = TurretAngle.getValueAsDouble()*360/10-SubsystemConstants.ShooterCenteredRotation;
        delta = Math.abs(angle-TurretRobotRelativeAngle);
        done = true;
        TurretAimed = false;
        if(delta > 4){
        setAngle(angle+SubsystemConstants.ShooterCenteredRotation);
        done = false;
        TurretAimed = false;
        m_RobotContainer.operatorController.setRumble(RumbleType.kBothRumble,delta/100);
        }
        else{TurretAimed = true;
             m_RobotContainer.operatorController.setRumble(RumbleType.kBothRumble,0);}
        return done;
    }

    public boolean TurretPIDFieldRelative(double angle){
        boolean done;
        done = TurretPIDRobotRelative(m_PoseEstimatorSubsystem.getRobotPose2d().getRotation().getDegrees()+angle);
        return done;
    }

    public double getTurretAngle(){
        double angle;
        angle = TurretAngle.getValueAsDouble()/10;
        angle = DegreesAngleClamp(angle*360);
        return angle;
    }

    public double getTurretAngleRobotRelative(){
        double angle;
        angle = getTurretAngle();
        angle = angle-SubsystemConstants.ShooterCenteredRotation;
        return angle;
    }
    

    //Full Controll
    public void AllStop(){
        StopFlywheelMotors();
        StopHoodMotor();
        StopTurretMotor();
    }
    
    //AimPose: the pose to aim at
    public boolean FullTurretAutoAim(Pose2d AimPose){
        Pose2d robotPose = m_PoseEstimatorSubsystem.getRobotPose2d();
        Translation2d chassisSpeed = new Translation2d(m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vxMetersPerSecond
                                                      ,m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vyMetersPerSecond)
                                                      .rotateBy(robotPose.getRotation());
        double distanceToHub = getDistanceToHub(AimPose);
        double desiredHoodAngle = LimelightConstants.TurretHoodInterpolatorDEG.get(distanceToHub);
        double desiredRPM = LimelightConstants.TurretFlywheelInterpolatorRPM.get(distanceToHub);
        Rotation2d desiredTurretAngle = new Rotation2d(-Math.atan2(AimPose.getY()-chassisSpeed.getY()*0.25-robotPose.getY(),AimPose.getX()-chassisSpeed.getX()*0.25-robotPose.getX()));
        TurretPIDFieldRelative(desiredTurretAngle.getDegrees());
        RunFlywheelMotor(desiredRPM);
        //HoodPID(desiredHoodAngle, 0.1, 0.1, tolerance);

        if(TurretAimed){
            StopTurretMotor();
            return true;
        }
        else{              
            return false;
        }
    }
    
    public double getDistanceToHub(Pose2d hubPose2d){
        Pose2d robotPose = m_PoseEstimatorSubsystem.getRobotPose2d();
        Translation2d chassisSpeed = new Translation2d(m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vxMetersPerSecond
                                                      ,m_PoseEstimatorSubsystem.getDrivetrain().getState().Speeds.vyMetersPerSecond)
                                                      .rotateBy(robotPose.getRotation());
        double distanceToHub;
        distanceToHub = (hubPose2d.relativeTo(robotPose).getTranslation().minus(chassisSpeed)).getNorm();
        System.out.println(distanceToHub);
        return distanceToHub;
    }

    public void continueousTurretAutoAim(boolean enable,boolean redAlliance){
        constantGoalRedAlliance = redAlliance;
        constantAutoAim = enable;
    }

    public void continueousTurretSlowblowAim(boolean enable,boolean redAlliance){
        constantGoalRedAlliance = redAlliance;
        constantSnowblow = enable;
    }

    @Override
    public void periodic() {
        m_RobotContainer.operatorController.setRumble(RumbleType.kBothRumble, 0);
        //Set own values
        FlywheelRPM = getFlywheelSpeed();
        getTurretAngle();
        SmartDashboard.getEntry("TurretAimed").setBoolean(TurretAimed);
        SmartDashboard.getEntry("Shooter RPM").setNumber(FlywheelRPM);
        //update motor status signals
        BaseStatusSignal.refreshAll(TurretAngle,
                                    m_FlywheelMotor.getVelocity(),
                                    m_HoodMotor.getPosition(),
                                    m_TurretMotor.getVelocity());
        //feed the pose estimator numbers
        m_PoseEstimatorSubsystem.putShooterRotation(SubsystemConstants.ShooterCenteredRotation-getTurretAngle());
        m_PoseEstimatorSubsystem.putShooterRotationalVelocity(m_TurretMotor.getVelocity().getValueAsDouble()*36);
        //update smartdashboard
        SmartDashboard.updateValues();
        //Do constant PIDs when issued
        //if(constantAutoAim){FullTurretAutoAim(constantGoalRedAlliance ?LimelightConstants.RedHubPose2d : LimelightConstants.BlueHubPose2d, 5); constantSnowblow = false;}
        //if(constantSnowblow){TurretPIDFieldRelative(constantGoalRedAlliance ? 0:180); constantAutoAim = false;}

    }

    //handle Turret Simulation
    public void simulationPeriodic() {
    m_TurretSim.setInput(m_TurretMotor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    m_TurretSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        m_TurretSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = (m_TurretSim.getAngleRads()*10)/(2*Math.PI);
    double motorVelocity = (m_TurretSim.getVelocityRadPerSec() * 10)/(2*Math.PI);

    m_TurretMotor.getSimState().setRawRotorPosition(motorPosition);
    m_TurretMotor.getSimState().setRotorVelocity(motorVelocity);
  }
}
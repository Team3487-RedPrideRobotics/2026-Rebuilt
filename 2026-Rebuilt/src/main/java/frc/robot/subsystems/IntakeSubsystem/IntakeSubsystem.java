package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    
    public TalonFX m_pivotMotor;
    private TalonFX m_intakeMotor;

    DutyCycleOut        m_pivotMotorRequest;
    DutyCycleOut        m_intakeMotorRequest;
    PositionDutyCycle   m_pivotPIDRequest;

    double customIntakeSpeed;

    double intakeAngle;

    public IntakeSubsystem(){
    
    m_pivotMotor = new TalonFX(SubsystemConstants.IntakePivotKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS); //initalize motors
    m_intakeMotor = new TalonFX(SubsystemConstants.IntakeKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

    m_pivotMotorRequest  = new DutyCycleOut(0.0);
    m_pivotPIDRequest    = new PositionDutyCycle(0);
    m_intakeMotorRequest = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);
    
    m_pivotMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakePivotKrakenInverted));
    m_intakeMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakeKrakenInverted));

    TalonFXConfiguration m_intakePivotConfig = new TalonFXConfiguration(); //make configs
    TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();

    Slot0Configs m_pivotPIDs = new Slot0Configs(); //set PIDs
    m_pivotPIDs.kP = 0.25;
    m_pivotPIDs.kI = 0;
    m_pivotPIDs.kD = 0;

    SoftwareLimitSwitchConfigs softLimitsIntakePivot = m_intakePivotConfig.SoftwareLimitSwitch; //set limits
        softLimitsIntakePivot.ForwardSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitTop;
        softLimitsIntakePivot.ForwardSoftLimitEnable = true;
        softLimitsIntakePivot.ReverseSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitBototm;
        softLimitsIntakePivot.ReverseSoftLimitEnable = true;

    m_intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //set breaking

    SoftwareLimitSwitchConfigs softLimitsIntake = m_intakeConfig.SoftwareLimitSwitch;
    softLimitsIntake.ForwardSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitTop;
    softLimitsIntake.ForwardSoftLimitEnable = false;
    softLimitsIntake.ReverseSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitBototm;
    softLimitsIntake.ReverseSoftLimitEnable = false;

    m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_pivotMotor.getConfigurator().apply(m_intakePivotConfig); //apply configs
    m_intakeMotor.getConfigurator().apply(m_intakeConfig);
    

    m_pivotMotor.setPosition(0);

    SmartDashboard.putNumber("Intake Angle", intakeAngle);

    SmartDashboard.putNumber("Custom Intake Speed", customIntakeSpeed);

    }

    public void RunMotorPivot(double speed){
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() > SubsystemConstants.IntakePiviotHardLimitTop ? speed : speed;
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() < SubsystemConstants.IntakePiviotHardLimitBototm ? speed : speed; 
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public void StopMotorPivot(){
        m_pivotMotorRequest.withOutput(0);
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public boolean IntakePiviotPID(double goalValueDeg, double threshold){
        double delta = Math.abs(goalValueDeg*360/80) - Math.abs(intakeAngle);
        if(Math.abs(delta) >= threshold){
            m_pivotPIDRequest.withPosition(goalValueDeg/360*80);
            return false;
    }   else {
            StopMotorPivot();
            return true;
    }
    }

    public double TurretTurnsToDeg(double turns){
        return(((turns*SubsystemConstants.IntakePivotGearRatio)-Math.floor(turns*SubsystemConstants.IntakePivotGearRatio))/360); 
    }

    public void RunIntake(double speed){
        if(customIntakeSpeed != 0){
            m_intakeMotorRequest.withOutput(customIntakeSpeed);
        }
        else{
        m_intakeMotorRequest.withOutput(speed);
        }
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void StopIntake(){
        m_intakeMotorRequest.withOutput(0);
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void periodic() {
        customIntakeSpeed = SmartDashboard.getNumber("Custom Intake Speed", 0);
        BaseStatusSignal.refreshAll(m_pivotMotor.getPosition());
        intakeAngle = m_pivotMotor.getPosition().getValueAsDouble();
    }

}
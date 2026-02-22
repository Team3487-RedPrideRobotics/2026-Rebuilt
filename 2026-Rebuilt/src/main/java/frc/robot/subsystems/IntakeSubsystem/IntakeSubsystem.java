package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    
    private TalonFX m_pivotMotor;
    private TalonFX m_intakeMotor;

    DutyCycleOut m_pivotMotorRequest;
    DutyCycleOut m_intakeMotorRequest;


    public IntakeSubsystem(){
    
    m_pivotMotor = new TalonFX(SubsystemConstants.IntakePivotKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
    m_intakeMotor = new TalonFX(SubsystemConstants.IntakeKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

    m_pivotMotorRequest = new DutyCycleOut(0.0);
    m_intakeMotorRequest = new DutyCycleOut(0.0);
    
    m_pivotMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakePivotKrakenInverted));
    m_intakeMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakeKrakenInverted));
    
    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void RunMotorPivot(double speed){
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() > SubsystemConstants.IntakePiviotHardLimitTop ? -speed : speed;
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() < SubsystemConstants.IntakePiviotHardLimitBototm ? -speed : speed; 
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public void StopMotorPivot(){
        m_pivotMotorRequest.Output = 0;
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public boolean IntakePiviotPID(double goalValue,double limit, double kP, double threshold){
        double delta = Math.abs(goalValue) - Math.abs(m_pivotMotor.getPosition().getValueAsDouble());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunMotorPivot(speed);
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
        m_intakeMotorRequest.Output = speed;
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void StopIntake(){
        m_intakeMotorRequest.Output = 0;
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

}
package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.generated.SubsystemConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    
    private TalonFX m_pivotMotor;
    private TalonFX m_intakeMotor;

    DutyCycleOut m_pivotMotorRequest;
    DutyCycleOut m_intakeMotorRequest;


    public IntakeSubsystem(){
    
    m_pivotMotor = new TalonFX(SubsystemConstants.IntakePivotKrakenCANID);
    m_intakeMotor = new TalonFX(SubsystemConstants.IntakeKrakenCANID);

    m_pivotMotorRequest = new DutyCycleOut(0.0);
    m_intakeMotorRequest = new DutyCycleOut(0.0);
    
    
    m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
    m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void RunMotorPivot(double speed){
        m_pivotMotorRequest.Output = speed;
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

// turn limits 

public double TurretTurnsToDeg(double turns){
        return(((turns*SubsystemConstants.IntakePivotGearRatio)-Math.floor(turns*SubsystemConstants.IntakePivotGearRatio))/360); 
    }


            public void IntakePiviotPID(double speed) {

        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() > SubsystemConstants.IntakePiviotHardLimitTop ? -speed : speed;
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() < SubsystemConstants.IntakePiviotHardLimitBototm ? -speed : speed; 
        m_pivotMotor.setControl(m_pivotMotorRequest);
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
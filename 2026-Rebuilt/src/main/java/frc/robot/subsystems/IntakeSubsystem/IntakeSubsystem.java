package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    
    private TalonFX m_pivotMotor;
    private TalonFX m_intakeMotor;

    DutyCycleOut m_pivotMotorRequest;
    DutyCycleOut m_intakeMotorRequest;

    public IntakeSubsystem(){
    
    m_pivotMotor = new TalonFX(0);
    m_intakeMotor = new TalonFX(1);

    m_pivotMotorRequest = new DutyCycleOut(0.0);
    m_intakeMotorRequest = new DutyCycleOut(0.0);
    

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

    public void RunIntake(double speed){
        m_intakeMotorRequest.Output = speed;
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void StopIntake(){
        m_intakeMotorRequest.Output = 0;
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }




}
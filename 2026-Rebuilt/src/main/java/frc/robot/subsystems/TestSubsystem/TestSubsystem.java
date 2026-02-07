
//Test subsystemm that makes Motors spin

package frc.robot.subsystems.TestSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase{
    
    private TalonFX m_Motor;

    DutyCycleOut m_motorRequest;

    public TestSubsystem(){
    
    m_Motor = new TalonFX(0);

    m_motorRequest = new DutyCycleOut(0.0);
    
    }

    public void RunMotor(double speed){
        m_motorRequest.Output = speed;
        m_Motor.setControl(m_motorRequest);
    }

    public void StopMotors(){
        m_motorRequest.Output = 0;
        m_Motor.setControl(m_motorRequest);
    }

    public boolean SubsystemPID(double goalValue,double limit, double kP, double threshold){
        double delta = Math.abs(goalValue) - Math.abs(m_Motor.getPosition().getValueAsDouble());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunMotor(speed);
            return false;
    }   else {
            StopMotors();
            return true;
    }


}}

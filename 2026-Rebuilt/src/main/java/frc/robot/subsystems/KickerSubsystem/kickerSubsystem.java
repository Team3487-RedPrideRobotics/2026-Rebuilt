package frc.robot.subsystems.KickerSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class kickerSubsystem extends SubsystemBase {
    
     private TalonFX m_Motor;

    DutyCycleOut m_motorRequest;

    public kickerSubsystem(){
    
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

   
}

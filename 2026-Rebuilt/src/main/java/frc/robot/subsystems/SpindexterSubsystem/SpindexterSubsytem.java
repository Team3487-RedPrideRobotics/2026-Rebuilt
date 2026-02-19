package frc.robot.subsystems.SpindexterSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class SpindexterSubsytem extends SubsystemBase {

         private TalonFX m_Motor;

    DutyCycleOut m_motorRequest;

    public SpindexterSubsytem(){
    
    m_Motor = new TalonFX(SubsystemConstants.SpindexterKrakenCANID);

    m_motorRequest = new DutyCycleOut(0.0);

    m_Motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.SpindexterKrakenInverted));

    m_Motor.setNeutralMode(NeutralModeValue.Coast);
    
    }

    public void RunMotor(double speed){
        m_motorRequest.Output = speed;
        m_Motor.setControl(m_motorRequest);
    }

    public void StopMotors(){
        m_motorRequest.Output = 0;
        m_Motor.stopMotor();
    }
    
}

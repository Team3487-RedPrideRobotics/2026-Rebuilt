package frc.robot.subsystems.KickerSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.generated.SubsystemConstants;

public class kickerSubsystem extends SubsystemBase {
    
     private TalonFX m_Motor;

    DutyCycleOut m_motorRequest;

    public kickerSubsystem(){
    
    m_Motor = new TalonFX(SubsystemConstants.KickerKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

    m_motorRequest = new DutyCycleOut(0.0);

    m_Motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.KickerKrakenInverted));

    m_Motor.setNeutralMode(NeutralModeValue.Brake);
    
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

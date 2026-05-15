package frc.robot.subsystems.KickerSubsystem;

import java.util.Map;

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

    // instances a new motor with the CAN Id and CAN Bus from the supplied constants
    m_Motor = new TalonFX(SubsystemConstants.KickerKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);
    
    //makes a new request for a duty cycle without worrying about the hardware limits
    m_motorRequest = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);

    //Gets the configurator and tells it to set if the motor is inverted based 
    m_Motor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.KickerKrakenInverted));

    //sets the motor to turn on its brakes when commanded to stop
    m_Motor.setNeutralMode(NeutralModeValue.Brake);
    
    }

    //runs the motor at the set speed
    public void RunMotor(double speed){
        m_motorRequest.withOutput(speed);
        m_Motor.setControl(m_motorRequest);
        double[] numbers = {1,2,3};
    }

    //Stops the rotation of the motors
    public void StopMotors(){
        m_motorRequest.withOutput(0);
        m_Motor.stopMotor();
    }
}

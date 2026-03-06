package frc.robot.subsystems.ClimberSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    

    private TalonFX m_ClimbMotor;

    DutyCycleOut m_ClimbMotorRequest;

    private int direction;
    
    public ClimberSubsystem(){

    m_ClimbMotor = new TalonFX(SubsystemConstants.ClimberKrakenCANID, SubsystemConstants.SUBSYSTEM_BUS);
        
    m_ClimbMotorRequest = new DutyCycleOut(0);

    m_ClimbMotor.setNeutralMode(NeutralModeValue.Brake);

    }

// the pid for the motor going up and down
    public void RunClimbMotor(double speed) {
        m_ClimbMotorRequest.Output = speed;
    //hard stops
        //m_ClimbMotorRequest.Output = m_ClimbMotor.getPosition().getValueAsDouble() > SubsystemConstants.ClimberClimbHardLimitTop ? -speed: speed;
        //m_ClimbMotorRequest.Output = m_ClimbMotor.getPosition().getValueAsDouble() < SubsystemConstants.ClimberClimbHardLimitBottom ? -speed : speed; 
        m_ClimbMotor.setControl(m_ClimbMotorRequest);
    }


    public void StopClimbMotor(){
        m_ClimbMotorRequest.Output = 0;
        m_ClimbMotor.stopMotor();
    }

    public boolean ClimbMotorPID(double goalValue,double limit, double kP, double threshold){
        double delta = Math.abs(goalValue) - Math.abs(m_ClimbMotor.getPosition().getValueAsDouble());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunClimbMotor(speed);
            return false;
    }   else {
            StopClimbMotor();
            return true;
    }

    }

// setting the direction for the motor becuase its on a winch
    public void SetDirection(int Direction){
        direction = Direction;
        m_ClimbMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(
            Direction == 1? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive));
    }

    public int GeDirection(){
            return direction;
}

    public double detectClimbMotorCurrent() {
    
        return m_ClimbMotor.getStatorCurrent().getValueAsDouble();
    }

}

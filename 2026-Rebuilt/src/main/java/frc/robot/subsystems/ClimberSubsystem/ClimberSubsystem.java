package frc.robot.subsystems.ClimberSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    
    private TalonFX m_holdMotor;
    private TalonFX m_ClimbMotor;
    private TalonFX m_ClimbPivotMotor;

    DutyCycleOut m_holdMotorRequest;
    DutyCycleOut m_ClimbMotorRequest;
    DutyCycleOut m_ClimbPivotMotorRequest;

    private int direction;
    
    public ClimberSubsystem(){
        
        m_holdMotorRequest = new DutyCycleOut(0);
        m_ClimbMotorRequest = new DutyCycleOut(0);
        m_ClimbPivotMotorRequest = new DutyCycleOut(0);

    }

    public void RunHoldMotor(double speed){
        m_holdMotorRequest.Output = speed;
        m_holdMotor.setControl(m_holdMotorRequest);
    }

    public void StopHoldMotor(){
        m_holdMotorRequest.Output = 0;
        m_holdMotor.stopMotor();
    }

    public boolean HoldMotorPID(double goalValue,double limit, double kP, double threshold){
        double delta = Math.abs(goalValue) - Math.abs(m_holdMotor.getPosition().getValueAsDouble());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunHoldMotor(speed);
            return false;
    }   else {
            StopHoldMotor();
            return true;
    }

    }


        public void RunClimbMotor(double speed) {

        m_ClimbMotorRequest.Output = m_ClimbMotor.getPosition().getValueAsDouble() > SubsystemConstants.ClimberClimbHardLimitTop ? -speed : speed;
        m_ClimbMotorRequest.Output = m_ClimbMotor.getPosition().getValueAsDouble() < SubsystemConstants.ClimberClimbHardLimitBottom ? -speed : speed; 
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

     public void RunClimbPivotMotor(double speed){
        m_ClimbPivotMotorRequest.Output = speed;
        m_ClimbPivotMotor.setControl(m_ClimbPivotMotorRequest);
    }

    public void StopClimbPivotMotor(){
        m_ClimbPivotMotor.stopMotor();
    }

    public boolean ClimbPivotMotorPID(double goalValue,double limit, double kP, double threshold){
        double delta = Math.abs(goalValue) - Math.abs(m_ClimbPivotMotor.getPosition().getValueAsDouble());
        if(Math.abs(delta) >= threshold){
            var speed = -delta*kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunClimbPivotMotor(speed);
            return false;
    }   else {
            StopClimbPivotMotor();
            return true;
    }

    }

    public void SetDirection(int Direction){
        direction = Direction;
    }

    public int GeDirection(){
            return direction;
}

}

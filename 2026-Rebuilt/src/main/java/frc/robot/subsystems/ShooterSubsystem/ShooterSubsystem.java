
//Test subsystemm that makes Motors spin

package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;
    private TalonFX m_TurretMotor;
    private TalonFX m_HoodMotor;

    private DutyCycleOut m_FlywheelMotorRequest;
    private DutyCycleOut m_TurretMotorRequest;
    private DutyCycleOut m_HoodMotorRequest;

    Alert TurretRingOverrun = new Alert("Turret ring overrun!", AlertType.kWarning);

    public ShooterSubsystem() {

        m_FlywheelMotor = new TalonFX(SubsystemConstants.ShooterFlywheelKrakenCANID);
        m_TurretMotor = new TalonFX(SubsystemConstants.ShooterTurretKrakenCANID);
        m_HoodMotor = new TalonFX(SubsystemConstants.ShooterHoodKrakenCANID);

        m_FlywheelMotorRequest = new DutyCycleOut(0.0);
        m_TurretMotorRequest = new DutyCycleOut(0.0);
        m_HoodMotorRequest = new DutyCycleOut(0.0);

    }

    //Flywheel Control
    public void RunFlywheelMotor(double speed) {
        m_FlywheelMotorRequest.Output = speed;
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    public void StopFlywheelMotors() {
        m_FlywheelMotorRequest.Output = 0;
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    //Hood Control
    public void RunHoodMotor(double speed) {

        m_HoodMotorRequest.Output = m_HoodMotor.getPosition().getValueAsDouble() > SubsystemConstants.ShooterHoodHardLimitTop ? -speed : speed;
        m_HoodMotorRequest.Output = m_HoodMotor.getPosition().getValueAsDouble() < SubsystemConstants.ShooterHoodHardLimitBottom ? -speed : speed; 
        m_HoodMotor.setControl(m_HoodMotorRequest);
    }

    public void StopHoodMotor() {
        m_HoodMotorRequest.Output = 0;
        m_HoodMotor.setControl(m_HoodMotorRequest);
    }

    //Goal in turns, Limit in max speed, kP as P value, threshold as in tolerance
    public boolean HoodPID(double goalValue, double limit, double kP, double threshold) {
        double delta = Math.abs(goalValue) - Math.abs(m_HoodMotor.getPosition().getValueAsDouble());
        if (Math.abs(delta) >= threshold) {
            var speed = -delta * kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunHoodMotor(speed);
            return false;
        } else {
            StopHoodMotor();
            return true;
        }

    }

    //Turret Control
    public void RunTurretMotor(double speed) {

        m_TurretMotorRequest.Output = m_TurretMotor.getPosition().getValueAsDouble() > SubsystemConstants.ShooterHoodHardLimitTop ? -speed : speed;
        m_TurretMotorRequest.Output = m_TurretMotor.getPosition().getValueAsDouble() < SubsystemConstants.ShooterHoodHardLimitBottom ? -speed : speed; 
        m_TurretMotor.setControl(m_TurretMotorRequest);
    }

    public void StopTurretMotor() {
        m_TurretMotorRequest.Output = 0;
        m_TurretMotor.setControl(m_TurretMotorRequest);
    }

    public boolean TurretPID(double goalValue, double limit, double kP, double threshold) {
        if(goalValue >= SubsystemConstants.SooterTurretHardLimitTop || goalValue <= SubsystemConstants.SooterTurretHardLimitBottom){
        TurretRingOverrun.set(false);
        double delta = Math.abs(goalValue) - Math.abs(m_TurretMotor.getPosition().getValueAsDouble());
        if (Math.abs(delta) >= threshold) {
            var speed = -delta * kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunTurretMotor(speed);
            return false;
        } else {
            StopTurretMotor();
            return true;
        }
    }
    else{
        System.err.println("Turret Ring Overrun!");
        TurretRingOverrun.set(true);
        return(false);
    }
    }
}

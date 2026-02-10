
//Test subsystemm that makes Motors spin

package frc.robot.subsystems.ShooterSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX m_FlywheelMotor;

    DutyCycleOut m_FlywheelMotorRequest;

    public ShooterSubsystem() {

        m_FlywheelMotor = new TalonFX(0);

        m_FlywheelMotorRequest = new DutyCycleOut(0.0);

    }

    public void RunMotor(double speed) {
        m_FlywheelMotorRequest.Output = speed;
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    public void StopMotors() {
        m_FlywheelMotorRequest.Output = 0;
        m_FlywheelMotor.setControl(m_FlywheelMotorRequest);
    }

    public boolean SubsystemPID(double goalValue, double limit, double kP, double threshold) {
        double delta = Math.abs(goalValue) - Math.abs(m_FlywheelMotor.getPosition().getValueAsDouble());
        if (Math.abs(delta) >= threshold) {
            var speed = -delta * kP;
            speed = Math.abs(speed) > limit ? limit * Math.signum(speed) : speed;
            RunMotor(speed);
            return false;
        } else {
            StopMotors();
            return true;
        }

    }
}

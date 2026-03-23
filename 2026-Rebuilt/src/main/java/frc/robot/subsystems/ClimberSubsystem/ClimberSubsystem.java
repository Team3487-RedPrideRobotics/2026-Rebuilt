package frc.robot.subsystems.ClimberSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class ClimberSubsystem extends SubsystemBase {
    
    private TalonFX m_ClimbMotor;

    DutyCycleOut m_ClimbMotorRequest;

    ElevatorSim m_ClimberSim;
    Mechanism2d m_ClimberSimMech = new Mechanism2d(3, 3);
    MechanismRoot2d m_ClimberSimRoot = m_ClimberSimMech.getRoot("ClimberArm", 2, 1);
    MechanismLigament2d m_ClimberSimPivot = m_ClimberSimRoot.append(new MechanismLigament2d("Arm", 0.4, 90));

    private int direction;
    
    public ClimberSubsystem(){

    m_ClimberSim = new ElevatorSim(DCMotor.getKrakenX60(1), 
                                    15, 
                                    0.1, 
                                    0.02, 
                                    0.3, 
                                    0.6, 
                                    false, 
                                    direction);

    m_ClimbMotor = new TalonFX(SubsystemConstants.ClimberKrakenCANID, SubsystemConstants.SUBSYSTEM_BUS);
    
    m_ClimbMotorRequest = new DutyCycleOut(0);

    m_ClimbMotor.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.putData("Climber Sim", m_ClimberSimMech);
    }

// the pid for the motor going up and down
    public void RunClimbMotor(double speed) {
        m_ClimbMotorRequest.Output = speed;
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

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_ClimbMotor.getPosition());

    }

    @Override
    public void simulationPeriodic() {
    m_ClimberSim.setInput(m_ClimbMotor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    m_ClimberSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        m_ClimberSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = ((m_ClimberSim.getPositionMeters()-0.3/0.3)*15)/(2*Math.PI);
    double motorVelocity = (m_ClimberSim.getVelocityMetersPerSecond() * 15)/(2*Math.PI);

    m_ClimbMotor.getSimState().setRawRotorPosition(motorPosition);
    m_ClimbMotor.getSimState().setRotorVelocity(motorVelocity);
    m_ClimberSimPivot.setLength(m_ClimberSim.getPositionMeters());
    }
}

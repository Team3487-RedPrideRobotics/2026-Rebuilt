package frc.robot.subsystems.IntakeSubsystem;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.SubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    
    public TalonFX m_pivotMotor;
    private TalonFX m_intakeMotor;

    DutyCycleOut        m_pivotMotorRequest;
    DutyCycleOut        m_intakeMotorRequest;
    PositionDutyCycle   m_pivotPIDRequest;

    double customIntakeSpeed;

    double intakeAngle;

    SingleJointedArmSim m_IntakeSim;
    Mechanism2d m_IntakeSimMech = new Mechanism2d(3, 3);
    MechanismRoot2d m_IntakeSimRoot = m_IntakeSimMech.getRoot("IntakePivot", 2, 1);
    MechanismLigament2d m_IntakeSimPivot = m_IntakeSimRoot.append(new MechanismLigament2d("pivot", 0.4, 90));

    public IntakeSubsystem(){

        m_IntakeSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), 
        80,//gear ratio
        1, // Arm moment of inertia
        0.3, // Arm length (m)
        Units.degreesToRadians(0), // Min angle of the motor (deg)
        Units.degreesToRadians(90), // Max angle of the subsystem(deg)
        false, // Simulate gravity No; Braking makes it not fall ever
        Units.degreesToRadians(0) // Starting position (rad)
        );
    
    m_pivotMotor = new TalonFX(SubsystemConstants.IntakePivotKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS); //initalize motors
    m_intakeMotor = new TalonFX(SubsystemConstants.IntakeKrakenCANID,SubsystemConstants.SUBSYSTEM_BUS);

    m_pivotMotorRequest  = new DutyCycleOut(0.0);
    m_pivotPIDRequest    = new PositionDutyCycle(0);
    m_intakeMotorRequest = new DutyCycleOut(0.0).withIgnoreHardwareLimits(true);
    
    m_pivotMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakePivotKrakenInverted));
    m_intakeMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(SubsystemConstants.IntakeKrakenInverted));

    TalonFXConfiguration m_intakePivotConfig = new TalonFXConfiguration(); //make configs
    TalonFXConfiguration m_intakeConfig = new TalonFXConfiguration();

    //intake config
    Slot0Configs m_pivotPIDs = new Slot0Configs(); //set PIDs
    m_pivotPIDs.kP = 2;
    m_pivotPIDs.kI = 0;
    m_pivotPIDs.kD = 0.5;
    m_intakePivotConfig.Slot0 = m_pivotPIDs;

    SoftwareLimitSwitchConfigs softLimitsIntakePivot = m_intakePivotConfig.SoftwareLimitSwitch; //set limits
        softLimitsIntakePivot.ForwardSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitTop;
        softLimitsIntakePivot.ForwardSoftLimitEnable = true;
        softLimitsIntakePivot.ReverseSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitBototm;
        softLimitsIntakePivot.ReverseSoftLimitEnable = true;

    m_intakePivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake; //set breaking

    //Intake roller config
    SoftwareLimitSwitchConfigs softLimitsIntake = m_intakeConfig.SoftwareLimitSwitch;
    softLimitsIntake.ForwardSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitTop;
    softLimitsIntake.ForwardSoftLimitEnable = false;
    softLimitsIntake.ReverseSoftLimitThreshold = SubsystemConstants.IntakePiviotHardLimitBototm;
    softLimitsIntake.ReverseSoftLimitEnable = false;

    m_intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    CurrentLimitsConfigs currentLimitsIntake = m_intakeConfig.CurrentLimits;
        currentLimitsIntake.withStatorCurrentLimit(Amps.of(80));
        currentLimitsIntake.withSupplyCurrentLimit(Amps.of(20));

    m_pivotMotor.getConfigurator().apply(m_intakePivotConfig); //apply configs
    m_intakeMotor.getConfigurator().apply(m_intakeConfig);
    
    m_pivotMotor.setPosition(0);

    SmartDashboard.putData("Intake Pivot", m_IntakeSimMech);
    }

    public void RunMotorPivot(double speed){
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() > SubsystemConstants.IntakePiviotHardLimitTop ? speed : speed;
        m_pivotMotorRequest.Output = m_pivotMotor.getPosition().getValueAsDouble() < SubsystemConstants.IntakePiviotHardLimitBototm ? speed : speed; 
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public void StopMotorPivot(){
        m_pivotMotorRequest.withOutput(0);
        m_pivotMotor.setControl(m_pivotMotorRequest);
    }

    public boolean IntakePiviotPID(double goalValueDeg, double threshold){
        double delta = Math.abs(goalValueDeg/360*80 - m_pivotMotor.getPosition().getValueAsDouble());
        System.out.println(delta);
        if(Math.abs(delta) >= threshold){
            m_pivotPIDRequest.withPosition(goalValueDeg/360*80);
            m_pivotMotor.setControl(m_pivotPIDRequest);
            return false;
    }   else {
            m_pivotMotor.stopMotor();
            return true;
    }
    }

    public void RunIntake(double speed){
        if(customIntakeSpeed != 0){
            m_intakeMotorRequest.withOutput(customIntakeSpeed);
        }
        else{
        m_intakeMotorRequest.withOutput(speed);
        }
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void StopIntake(){
        m_intakeMotorRequest.withOutput(0);
        m_intakeMotor.setControl(m_intakeMotorRequest);
    }

    public void periodic() {
        intakeAngle = m_pivotMotor.getPosition().getValueAsDouble();
        BaseStatusSignal.refreshAll(m_pivotMotor.getPosition());
        intakeAngle = m_pivotMotor.getPosition().getValueAsDouble();
    }

    //handle Turret Simulation
    public void simulationPeriodic() {
    m_IntakeSim.setInput(m_pivotMotor.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    m_IntakeSim.update(0.020);
    RoboRioSim.setVInVoltage(
      BatterySim.calculateDefaultBatteryLoadedVoltage(
        m_IntakeSim.getCurrentDrawAmps()
      )
    );

    double motorPosition = (m_IntakeSim.getAngleRads()*80)/(2*Math.PI);
    double motorVelocity = (m_IntakeSim.getVelocityRadPerSec() * 80)/(2*Math.PI);

    m_pivotMotor.getSimState().setRawRotorPosition(motorPosition);
    m_pivotMotor.getSimState().setRotorVelocity(motorVelocity);
    m_IntakeSimPivot.setAngle(180+m_IntakeSim.getAngleRads()/(2*Math.PI)*360);
  }

}
package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

public class SubsystemConstants {

    //subsystem Motor CAN Id's
    public final static int SpindexterKrakenCANID = 5;
    public final static int KickerKrakenCANID = 4;
    public final static int IntakeKrakenCANID = 6;
    public final static int IntakePivotKrakenCANID = 7;
    public final static int ShooterFlywheelKrakenCANID = 17;
    public final static int ShooterTurretKrakenCANID = 3000;
    public final static int ShooterHoodKrakenCANID = 46;
    public final static int ClimberKrakenCANID = 0;
    public final static int ClimerHoldKrakenCANID = 1;
    public final static int ClimberPivotKrakenCANID = 2;

    public final static CANBus SUBSYSTEM_BUS = new CANBus("Canivore0");

    //diretions for every motor
    public final static InvertedValue SpindexterKrakenInverted = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue KickerKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue IntakeKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue IntakePivotKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue ShooterFlywheelKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue ShooterTurretKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue ShooterHoodKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue ClimerHoldKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue ClimberPivotKrakenInverted = InvertedValue.Clockwise_Positive;

    //Hardlimits in turns
    public final static double ShooterHoodHardLimitTop = 5;
    public final static double ShooterHoodHardLimitBottom = 0;


    public final static double ClimberClimbHardLimitTop = 21.3; 
    public final static double ClimberClimbHardLimitBottom = 0; 

    public final static double ShooterTurretHardLimitTop = 5;
    public final static double ShooterTurretHardLimitBottom = 0;

    public final static double IntakePiviotHardLimitTop = 25.6; 
    public final static double IntakePiviotHardLimitBototm = 0; 

    //Gear ratios
    public final static double ShooterTurretGearRatio = 1/10;
    public final static double IntakePivotGearRatio = 1/80;
    //-25.1549 resting agnle
    //90 intaking angle

    //Manual Speed values
    public final static double TurretRotationSpeed = 0.1; // in Deg/(1/50 of a second)

    public final static double HoodRotationSpeed = 0.1;

    //Controller Deadbands
    public static class OperatorConstants{
        public final static double leftXdeadBand = 0.1;
        public final static double leftYdeadBand = 0.1;
    }


    //goku
    //vegeta
}

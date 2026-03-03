package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

public class SubsystemConstants {

    //subsystem Motor CAN Id's
    public final static int SpindexterKrakenCANID = 5;
    public final static int KickerKrakenCANID = 4;
    public final static int IntakeKrakenCANID = 6;
    public final static int IntakePivotKrakenCANID = 7;

    public final static CANBus SUBSYSTEM_BUS = new CANBus("Canivore0");

    public final static int ShooterFlywheelKrakenCANID = 8; 
    public final static int ShooterTurretKrakenCANID = 9; 
    public final static int ShooterHoodKrakenCANID = 10; 
    public final static int ClimberKrakenCANID = 0;
    public final static int ClimerHoldKrakenCANID = 1;
    public final static int ClimberPivotKrakenCANID = 2;

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
    public final static double ShooterHoodHardLimitTop = 500; //angled fully back (closest to flat)
    public final static double ShooterHoodHardLimitBottom = 1000; //angled fully forward (closest to vertical)

    public final static double ClimberClimbHardLimitTop = 210; //max extention
    public final static double ClimberClimbHardLimitBottom = 0; //fully retracted

    public final static double ShooterTurretHardLimitTop = -19*10; //the rightmost rotational hardstop
    public final static double ShooterTurretHardLimitBottom = 0; //the leftmost roational hardstop

    public final static double IntakePiviotHardLimitTop = 25.6; //fully deployed ,90 intaking angle
    public final static double IntakePiviotHardLimitBototm = 0; //stowed ,-25.1549 stowed angle(from vertical)

    //Gear ratios
    public final static double ShooterTurretGearRatio = 19/200;
    public final static double IntakePivotGearRatio = 1/80;

    //Manual Speed values
    public final static double TurretRotationSpeed = 0.1; // in Deg/(1/50 of a second)

    public final static double HoodRotationSpeed = 0.1;

    //The amount of turns of the motor required to turn from the left-most hardstop to face forward
    public final static double ShooterCenteredRotation = 5.26;

    //Controller Deadbands
    public static class OperatorConstants{
        public final static double leftXdeadBand = 0.1;
        public final static double leftYdeadBand = 0.1;
    }


    //goku
    //vegeta
}

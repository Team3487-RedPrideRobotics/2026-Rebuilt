package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;

public class SubsystemConstants {

    //subsystem Motor CAN Id's
    public final static int SpindexterKrakenCANID = 5;
    public final static int KickerKrakenCANID = 4;
    public final static int IntakeKrakenCANID = 6;
    public final static int IntakePivotKrakenCANID = 0;

    public final static CANBus SUBSYSTEM_BUS = new CANBus("Canivore0");

    public final static int ShooterFlywheelKrakenCANID = 17; 
    public final static int ShooterTurretKrakenCANID = 12; 
    public final static int ShooterHoodKrakenCANID = 11; 
    public final static int ClimberKrakenCANID = 1;

    //diretions for every motor
    public final static InvertedValue SpindexterKrakenInverted = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue KickerKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue IntakeKrakenInverted = InvertedValue.Clockwise_Positive;
    public final static InvertedValue IntakePivotKrakenInverted = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue ShooterFlywheelKrakenInverted = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue ShooterTurretKrakenInverted = InvertedValue.CounterClockwise_Positive;
    public final static InvertedValue ShooterHoodKrakenInverted = InvertedValue.Clockwise_Positive;

    //Hardlimits in turns
    public final static double ShooterHoodHardLimitTop = 70; //angled fully forward (closest to vertical)
    public final static double ShooterHoodHardLimitBottom = 0; //angled fully back (closest to flat)

    public final static double ClimberClimbHardLimitTop = 210; //max extention
    public final static double ClimberClimbHardLimitBottom = 0; //fully retracted

    public final static double ShooterTurretHardLimitTop = 5; //the rightmost rotational hardstop
    public final static double ShooterTurretHardLimitBottom = 0; //the leftmost roational hardstop

    public final static double IntakePiviotHardLimitTop = 25.6; //fully deployed ,90 intaking angle
    public final static double IntakePiviotHardLimitBototm = 0; //stowed ,-25.1549 stowed angle(from vertical)

    //Gear ratios
    public final static double ShooterTurretGearRatio = 20/200;
    public final static double ShooterHoodGearRatio = 25/1.5; //degrees rotated to degrees angled
    public final static double IntakePivotGearRatio = 1/80;

    //Manual Speed values
    public final static double TurretRotationSpeed = 0.25; // in Deg/(1/50 of a second or something)

    public final static double HoodRotationSpeed = 0.1; 

    //The amount of turns of the motor required to turn from the left-most hardstop to face forward
    public final static double ShooterCenteredRotation = 7.5;
    public final static double ShooterHoodLowestAngle = 30; //the angle in deg that the hood starts at

    //Controller Deadbands
    public static class OperatorConstants{
        public final static double leftXdeadBand = 0.1;
        public final static double leftYdeadBand = 0.1;
    }


    //goku
    //vegeta
}

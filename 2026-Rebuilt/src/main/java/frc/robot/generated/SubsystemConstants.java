package frc.robot.generated;

public class SubsystemConstants {

    //subsystem Motor CAN Id's
    public final static int SpindexterKrakenCANID = 40;
    public final static int KickerKrakenCANID = 41;
    public final static int IntakeKrakenCANID = 42;
    public final static int IntakePivotKrakenCANID = 43;
    public final static int ShooterFlywheelKrakenCANID = 44;
    public final static int ShooterTurretKrakenCANID = 45;
    public final static int ShooterHoodKrakenCANID = 46;
    public final static int ClimberKrakenCANID = 47;
    public final static int ClimerHoldKrakenCANID = 48;
    public final static int ClimberPivotKrakenCANID = 49;

    //in turns
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

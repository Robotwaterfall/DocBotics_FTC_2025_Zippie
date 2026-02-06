package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    public static final class OIConstants{


    }

    public static final class transferConstants{

        public static double transferMotorPower = 1.0;


    }

    @Config
    public static final class shooterConstants{
        public static double shooterPower = 0.54;

        public static double farZoneShooting = 0.64;
        public static double timeOutShooting = 0.35;
        public static double timeOutbetweenShoots = 2.5;

        @Config
        public static final class shooterPhysicsConstants{
            public static double theta = 60.0;      // launch angle
            public static double h = 1.0;           // goal height above floor (m)
            public static double y0 = 0.40;         // shooter exit height above floor (m)
            public static double efficiency = 0.6;  // Todo: 0.5 - 0.8 typical, tine this

            public static final double G_inchPerSecondSquared = 386.09;
            public static final double RIM_RADIUS_INCH = 18.3/2;     //center offset
            public static final double WHEEL_RADIUS_INCH = 2;   // 2in wheel = 0.0508m

            public static final double shooterTolerence = 5; //TODO: tune

        }

    }






    @Config
    public static final class limelightConstants{
        public static double llLockOnKp = 0.025;
        public static double lockOnTolerance = 5;

        // how many degrees back is your limelight rotated from perfectly vertical?
        public static double limelightMountAngleDegrees = 25.0; //TODO

        // distance from the center of the Limelight lens to the floor
        public static double limelightLensHeightInches = 20.0; //TODO

        // distance from the target to the floor
        public static double goalHeightInches = 1.0;


        public static double min_Tx = 4;
        public static double max_Tx = 25;
        public static double min_Shooter_Power = 0.54;
        public static double max_Shooter_Power = 0.64;

        public static double ShooterLockedZone = 8;

        public static double tyOffSet = -7;

    }


    @Config
    public static final class mecanumConstants{
        public static final double thresHold = 0.05;
        public static double clippedRotLockOnPower = 0.5;

        public static double strafeCorrection = 1.1;
        public static boolean isFieldCentric = true;

        public static final class encoderAutoConstants{

            public static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // REV ultraplanetary motor
            public static final double     DRIVE_GEAR_REDUCTION    = 20 ;     // External Gearing.
            public static final double     WHEEL_DIAMETER_INCHES   = 3 ;     // For figuring circumference
            public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

        }

    }

    public void periodic(){

    }





}

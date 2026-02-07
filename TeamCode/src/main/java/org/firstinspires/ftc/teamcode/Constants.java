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
        public static double shooterVelocity = 15;

        public static double farZoneShootingVelocity = 17;
        public static double timeOutShooting = 0.35;
        public static double timeOutbetweenShots = 1.0;
        public static double shooterTolerance = 0.3;

    }

    public static double llLockOnKp = 0.025;
    public static double lockOnDeadband = 5;


    @Config
    public static final class limelightConstants{

        public static double min_Tx = 4;
        public static double max_Tx = 25;
        public static double min_Shooter_Power = 0.54;
        public static double max_Shooter_Power = 0.64;

        public static double ShooterLockedZone = 8;

        public static double tyOffSet = -7;

    }


    public static final class mecanumConstants{
        public static final double thresHold = 0.05;
        public static double clippedRotLockOnPower = 0.5;

        public static double strafeCorrection = 1.1;
        public static boolean isFieldCentric = true;

        public static final class encoderAutoConstants{
            public static final double     DRIVE_SPEED             = 0.25;
            public static final double     TURN_SPEED              = 0.5;

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

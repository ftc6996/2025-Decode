package org.firstinspires.ftc.teamcode;

public interface Constants {

    //Alliance
    static final int kNOT_SET = -1;
    static final int kALLIANCE_RED = 2;
    static final int kALLIANCE_BLUE = 1;

    public interface Game {
        //April Tag constants
        static final int kTAG_ANY = -1;
        static final int kTAG_GOAL_BLUE = 20;
        static final int kTAG_OBELISK_GPP = 21;
        static final int kTAG_OBELISK_PGP = 22;
        static final int kTAG_OBELISK_PPG = 23;
        static final int kTAG_GOAL_RED = 24;

        static final double kTAG_DIMENSION_SIZE_IN = 6.5;
        static final double kTAG_DIMENSION_SIZE_MM = 165;

        static final double kTAG_TOP_GOAL_HEIGHT_IN = 38.75;
        static final double kTAG_CENTER_OFFSET_FROM_TOP_GOAL_IN = 9.25;

        static final int kPIPELINE_ALLIANCE_RED = 2;
        static final int kPIPELINE_ALLIANCE_BLUE = 1;
        static final int kPIPELINE_OBELISK = 0;
    }

    public interface Launcher {
        static final double kHOOD_MIN_POS = 1.0;
        static final double kHOOD_MAX_POS = .6;
        static final int kLAUNCHER_TARGET_VELOCITY_CLOSE = 850; //-80;
        static final int kLAUNCHER_TARGET_VELOCITY_FAR = 1150; //-80
        static final double kFEED_TIME_SECONDS = 1;
        static final double kFEED_OPEN_POS = 0;
        static final double kFEED_CLOSE_POS = 0.45;
        static final double kAUTO_TURN_SPEED = 1;
        //bore through encoder
        static final double kENCODER_CPR = 8192.0;
        static final double kDEGREES_PER_REVOLUTION = 360.0;
        static final double kDEGREES_PER_TICK = kDEGREES_PER_REVOLUTION/kENCODER_CPR;
    }

    public interface Drive {
        static final int kLEFT_FRONT = 0;
        static final int kRIGHT_FRONT = 1;
        static final int kLEFT_REAR = 2;
        static final int kLEFT_BACK = kLEFT_REAR;
        static final int kRIGHT_REAR = 3;
        static final int kRIGHT_BACK = kRIGHT_REAR;
        static final int kMAX_DRIVE_MOTORS = 4;

        static final double MAX_MOVE_SPEED = 1.0;
        static final double MIN_MOVE_SPEED = 0.2;
        static final double SPEED_INCREMENT = 0.1;

        static final int    DRIVER_MODE_FIELD = 0;
        static final int    DRIVER_MODE_ROBOT = 1;

        static final double TRACK_WIDTH_MM = 404;

        //SWYFT v2
        static final double WHEEL_DIAMETER_MM = 86;
        static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4; //3.385826771653543
        static final double COUNTS_PER_MOTOR_REV  = 28.0;
        static final double DRIVE_GEAR_REDUCTION = 12.7;
        static final double ENCODER_TICKS_PER_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION; //355.6
        static final double TICKS_PER_INCH = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
        static final double TICKS_PER_MM = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);


    }
}

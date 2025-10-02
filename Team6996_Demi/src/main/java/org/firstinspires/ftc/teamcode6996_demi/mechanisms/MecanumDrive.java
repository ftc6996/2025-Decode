package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrive {
    private DcMotor left_front_drive  = null;
    private DcMotor left_rear_drive   = null;
    private DcMotor right_front_drive = null;
    private DcMotor right_rear_drive  = null;

    public IMU imu;

    private RevHubOrientationOnRobot orientation = null;

    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    private double speedAdjustment = MAX_MOVE_SPEED;

    private final double COUNTS_PER_MOTOR_REV = 280; // AM-2964 with 40:1 gearbox
    private final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private boolean showTelemetry = false;
    private boolean isInitialized = false;
    public MecanumDrive()
    {
        //default to this standard orientation
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);;
    }

    public MecanumDrive(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection)
    {
        orientation = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
    }
    public void init(HardwareMap hardwareMap) {

        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
        isInitialized = true;
    }

    public void setMode(DcMotor.RunMode mode)
    {
        if (!isInitialized)
            return;

        left_front_drive.setMode(mode);
        left_rear_drive.setMode(mode);
        right_front_drive.setMode(mode);
        right_rear_drive.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode)
    {
        if (!isInitialized)
            return;

        left_front_drive.setZeroPowerBehavior(mode);
        left_rear_drive.setZeroPowerBehavior(mode);
        right_front_drive.setZeroPowerBehavior(mode);
        right_rear_drive.setZeroPowerBehavior(mode);
    }

    public void setMaxSpeed(double speed)
    {
        speedAdjustment = Range.clip(speed, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
    }

    public double getMaxSpeed()
    {
        return speedAdjustment;
    }

    public void setPower(double left_front, double right_front, double left_back, double right_back)
    {
        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double currentMaxSpeed = MAX_MOVE_SPEED;
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_back));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_back));

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        left_front /= currentMaxSpeed;
        right_front /= currentMaxSpeed;
        left_back /= currentMaxSpeed;
        right_back /= currentMaxSpeed;

        //flat adjustment, easy way to lower power without changing lots of code
        left_front *= speedAdjustment;
        right_front *= speedAdjustment;
        left_back *= speedAdjustment;
        right_back *= speedAdjustment;

        setRawPower(left_front, right_front, left_back, right_back);
    }

    public void setRawPower(double left_front, double right_front, double left_back, double right_back)
    {
        if (!isInitialized)
            return;

        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back);
    }

    public void setRawPower(double power)
    {
        if (!isInitialized)
            return;

        left_front_drive.setPower(power);
        left_rear_drive.setPower(power);
        right_front_drive.setPower(power);
        right_rear_drive.setPower(power);
    }
    /*
     * If we had a gyro and wanted to do field-oriented control, here
     * is where we would implement it.
     *
     * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
     * coordinate (strafe, drive), and we just rotate it by the gyro
     * reading minus the offset that we read in the init() method.
     * Some rough pseudocode demonstrating:
     *
     * if Field Oriented Control:
     *     get gyro heading
     *     subtract initial offset from heading
     *     convert heading to radians (if necessary)
     *     new strafe = strafe * cos(heading) - drive * sin(heading)
     *     new drive  = strafe * sin(heading) + drive * cos(heading)
     *
     * If you want more understanding on where these rotation formulas come
     * from, refer to
     * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
     */
    public void move(double drive, double strafe, double twist)
    {
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        // apply the calculated values to the motors.
        setPower(speeds[0], speeds[1], speeds[2], speeds[3]);
    }

    public void stop()
    {
        setRawPower(0,0,0,0);
    }

    public boolean isAllBusy()
    {
        return (left_front_drive.isBusy() &&
                right_front_drive.isBusy() &&
                left_rear_drive.isBusy() &&
                right_rear_drive.isBusy());
    }

    public boolean isAnyBusy()
    {
        return (left_front_drive.isBusy() ||
                right_front_drive.isBusy() ||
                left_rear_drive.isBusy() ||
                right_rear_drive.isBusy());
    }

    public int[] getAllPositions()
    {
        int [] pos = {left_front_drive.getCurrentPosition(),
                right_front_drive.getCurrentPosition(),
                left_rear_drive.getCurrentPosition(),
                right_rear_drive.getCurrentPosition()};
        return pos;
    }

    public void setTargetPosition(int [] pos){
        left_front_drive.setTargetPosition(pos[0]);
        right_front_drive.setTargetPosition(pos[1]);
        left_rear_drive.setTargetPosition(pos[2]);
        right_rear_drive.setTargetPosition(pos[3]);
    }
}

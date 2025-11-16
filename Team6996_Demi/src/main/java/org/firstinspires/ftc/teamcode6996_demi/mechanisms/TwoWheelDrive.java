package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

public class TwoWheelDrive {
    private DcMotor right_drive  = null;
    private DcMotor left_drive   = null;
    public IMU imu;
    private RevHubOrientationOnRobot orientation = null;
    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    private double speedAdjustment = MAX_MOVE_SPEED;

    private final double COUNTS_PER_MOTOR_REV = 280; // AM-2964 with 40:1 gearbox
    private final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    private boolean isInitialized = false;
    public TwoWheelDrive()
    {
        //default to this standard orientation
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
    }

    public TwoWheelDrive(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection)
    {
        orientation = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);
    }

    public void init(HardwareMap hardwareMap) {

        left_drive  = hardwareMap.get(DcMotor.class, "left_drive");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");


        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);

        left_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
        isInitialized = true;
    }

    public void setMode(DcMotor.RunMode mode)
    {
        if (!isInitialized)
            return;

        left_drive.setMode(mode);
        right_drive.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode)
    {
        if (!isInitialized)
            return;

        left_drive.setZeroPowerBehavior(mode);
        right_drive.setZeroPowerBehavior(mode);
    }

    public void setMaxSpeed(double speed)
    {
        speedAdjustment = Range.clip(speed, MIN_MOVE_SPEED, MAX_MOVE_SPEED);
    }

    public double getMaxSpeed()
    {
        return speedAdjustment;
    }

    public void setPower(double left_front, double right_front)
    {
        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double currentMaxSpeed = MAX_MOVE_SPEED;
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_front));

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        left_front /= currentMaxSpeed;
        right_front /= currentMaxSpeed;

        //flat adjustment, easy way to lower power without changing lots of code
        left_front *= speedAdjustment;
        right_front *= speedAdjustment;

        setRawPower(left_front, right_front);
    }

    public void setRawPower(double left_front, double right_front)
    {
        if (!isInitialized)
            return;

        left_drive.setPower(-left_front + 0.1);
        right_drive.setPower(-right_front);
    }

    public void setRawPower(double power)
    {
        if (!isInitialized)
            return;

        left_drive.setPower(-power);
        right_drive.setPower(-power);
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
                (drive  + twist),
                (drive  - twist),

        };
        // apply the calculated values to the motors.
        setPower(speeds[0], speeds[1]);
    }

    public void stop()
    {
        setRawPower(0,0);
    }

    public boolean isAllBusy()
    {
        return (left_drive.isBusy() &&
                right_drive.isBusy());

    }

    public boolean isAnyBusy()
    {
        return (left_drive.isBusy() ||
                right_drive.isBusy());
    }

    public int[] getAllPositions()
    {
        int [] pos = {left_drive.getCurrentPosition(),
                right_drive.getCurrentPosition(),
                0, 0};

        return pos;
    }

    public void setTargetPosition(int [] pos){
        left_drive.setTargetPosition(pos[0]);
        right_drive.setTargetPosition(pos[1]);
    }
}

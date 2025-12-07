package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    private DcMotor left_front_drive  = null;
    private DcMotor left_rear_drive   = null;
    private DcMotor right_front_drive = null;
    private DcMotor right_rear_drive  = null;
    private CRServo intake  = null;
    private ElapsedTime driveTimer = new ElapsedTime();
    private GoBildaPinpointDriver PinPoint;
    public IMU imu;
    final double TRACK_WIDTH_MM = 404;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));

    private RevHubOrientationOnRobot orientation = null;

    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    private double speedAdjustment = MAX_MOVE_SPEED;
/*
    private final double COUNTS_PER_MOTOR_REV = 280; // AM-2964 with 40:1 gearbox
    private final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
    private final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
*/
    private double countsPerMotorRev = 1120; //Demi AM-2964 with 40:1 gearbox
    private double wheelDiameter = 102;

    private boolean showTelemetry = false;
    private boolean isInitialized = false;

    private enum TurnState
    {
        IDLE,
        TURNING,
        COMPLETED;
    }
    private TurnState rotating = TurnState.IDLE;

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

        //intake = hardwareMap.get(CRServo.class, "servo0");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);

        PinPoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        PinPoint.setPosition(new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES,0));
        PinPoint.setOffsets(120,50,DistanceUnit.MM);
        PinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        PinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        isInitialized = true;
    }

    public void setWheelDiameter(double diameter)
    {
        wheelDiameter = diameter;
    }

    public void setMotorTicksPerRev(double ticks)
    {
        countsPerMotorRev = ticks;
    }

    public double getTicksPerMM()
    {
        return countsPerMotorRev / (wheelDiameter * Math.PI);
    }

    public double getTicksPerIN()
    {
        return getTicksPerMM() / 25.4;
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
                (drive - strafe + twist),
                (drive + strafe - twist),
                (drive + strafe + twist),
                (drive - strafe - twist)
        };
        // apply the calculated values to the motors.
        setPower(speeds[0], speeds[1], speeds[2], speeds[3]);
    }
    public boolean moveToPoint(double targetXInTiles, double targetYInTiles, double stopRangeInMm) {
        double XDone = 0;
        double YDone = 0;
        boolean done = false;
        while ((XDone + YDone) < 2) {
            PinPointUpdate();
            //robot.getPinpointPosition();
            //if(robot. != null){
            double slowSpeed = 0.0025;
            double stopRangeMm = stopRangeInMm;//Mm
            double targetXMm = targetXInTiles * 609.6;// Distance that the robot ends at in Mm
            double targetYMm = targetYInTiles * 609.6;// Distance that the robot ends at in Mm
            targetXMm = -targetXMm;
            //if (targetHeadingDeg < 0){
            //     targetHeadingDeg = -360-targetHeadingDeg;
            //}
            double currentXMm = getPinpointPosition().getX(DistanceUnit.MM);
            double currentYMm = getPinpointPosition().getY(DistanceUnit.MM);
            double XMm = targetXMm - currentXMm;
            double YMm = targetYMm - currentYMm;
            double XMmAbs = 0;
            double YMmAbs = 0;

            //slowCalculator{
            //Maybe later
            //}

            if ((Math.abs(targetXMm) - Math.abs(currentXMm)) <= stopRangeMm) {
                XMm = 0;
                XDone = 1;
            } else {
                XDone = 0;
                //TranslatorX{
                XMmAbs = Math.abs(XMm * slowSpeed);
                XMm = XMm * slowSpeed;
                if (XMmAbs <= 0.1) {
                    XMm = 0;
                }
                //}
            }
            if ((Math.abs(targetYMm) - Math.abs(currentYMm)) <= stopRangeMm) {
                YMm = 0;
                YDone = 1;
            } else {
                YDone = 0;
                //TranslatorY{
                YMmAbs = Math.abs(YMm * slowSpeed);
                YMm = YMm * slowSpeed;
                if (YMmAbs <= 0.1) {
                    YMm = 0;
                }
                //}
            }
            move(YMm, XMm, 0);


            if ((XDone + YDone) < 2) {
                done = true;
                return done;
            }
        }
        return done;
    }
    public static double angleError(double target, double current){
        double error = target - current;
        double errorr = (error + 180) % 360;
        if(errorr < 0){
            errorr = errorr + 360;
        }
        return errorr - 180;
    }
    public void turnToPoint(double targetHeadingInDeg, double stopTurnRangeInDeg)
    {

        double XDone = 0;
        double YDone = 0;
        while((XDone+YDone)<2){
            PinPointUpdate();
            //robot.getPinpointPosition();
            //if(robot. != null){
            double slowSpeed = 0.01;
            double stopTurnRangeDeg = stopTurnRangeInDeg;//deg
            double targetHeadingDeg = targetHeadingInDeg;// angle that the robot ends at in deg
            //if (targetHeadingDeg < 0){
            //     targetHeadingDeg = -360-targetHeadingDeg;
            //}
            double currentHeadingDeg = getPinpointPosition().getHeading(AngleUnit.DEGREES);
            double moveDeg = angleError(targetHeadingDeg,currentHeadingDeg);
            double thing = targetHeadingDeg -currentHeadingDeg;
            double moveDegAbs = 0;

            //slowCalculator{
            //Maybe later
            //}

            if ((Math.abs(targetHeadingDeg) - Math.abs(currentHeadingDeg))<= stopTurnRangeDeg){
                moveDeg = 0;
            }else{
                //TranslatorTurn{
                moveDegAbs = Math.abs(moveDeg*slowSpeed);
                moveDeg = moveDeg*slowSpeed;
                if(moveDegAbs <= 0.1) {
                    moveDeg = 0;
                }
                //}
            }
            move(0,0,moveDeg);
        }
    }
    /**
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    public boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        double TICKS_PER_MM = getTicksPerMM();
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        int [] pos = {(int) targetPosition, (int) targetPosition, (int) targetPosition, (int) targetPosition};
        setTargetPosition(pos);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setRawPower(speed);

        /*
         * Here we check if we are within tolerance of our target position or not. We calculate the
         * absolute error (distance from our setpoint regardless of if it is positive or negative)
         * and compare that to our tolerance. If we have not reached our target yet, then we reset
         * the driveTimer. Only after we reach the target can the timer count higher than our
         * holdSeconds variable.
         */
        if(Math.abs(targetPosition - left_front_drive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    public boolean rotateBy(double degrees)
    {
        boolean finished = false;
        //positive left
        //All angles are in the range of -180 degrees to 180 degrees.
        double heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double normalized = heading_deg % 360;
        double diff = (degrees - heading_deg + 540) % 360 - 180;

        double speed=.2; //positive speed, turn left
        //negative speed, turn right
        if (degrees < 0)
            speed *= -1;

        switch (rotating)
        {
            case IDLE:
            {
                imu.resetYaw();
                rotating = TurnState.TURNING;
                break;
            }
            case TURNING:
            {
                setRawPower(-speed, speed, -speed, speed);
                if (Math.abs(diff) < 1) {
                    rotating = TurnState.COMPLETED;
                }
                break;
            }
            case COMPLETED:
            {
                finished = true;
                stop();
                break;
            }
        }
        return finished;
    }

    public void intake(double power) {
        intake.setPower(power);
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that our wheels
         * need to travel, we just need to multiply the requested angle in radians by the radius
         * of our turning circle.
         */
        double TRACK_WIDTH_MM = 100;
        double targetMm = angleUnit.toRadians(angle)*(TRACK_WIDTH_MM/2);

        /*
         * We need to set the left motor to the inverse of the target so that we rotate instead
         * of driving straight.
         */
        double leftTargetPosition = -(targetMm*TICKS_PER_MM);
        double rightTargetPosition = targetMm*TICKS_PER_MM;


        left_front_drive.setTargetPosition((int) leftTargetPosition);
        right_front_drive.setTargetPosition((int) rightTargetPosition);
        left_rear_drive.setTargetPosition((int) leftTargetPosition);
        right_rear_drive.setTargetPosition((int) rightTargetPosition);

        left_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_front_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left_front_drive.setPower(speed);
        right_front_drive.setPower(speed);
        left_rear_drive.setPower(speed);
        right_rear_drive.setPower(speed);

        if((Math.abs(leftTargetPosition - left_front_drive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
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

    public int[] getAllTargetPositions()
    {
        int [] pos = {left_front_drive.getTargetPosition(),
                right_front_drive.getTargetPosition(),
                left_rear_drive.getTargetPosition(),
                right_rear_drive.getTargetPosition()};
        return pos;
    }

    public void setTargetPosition(int [] pos){
        left_front_drive.setTargetPosition(pos[0]);
        right_front_drive.setTargetPosition(pos[1]);
        left_rear_drive.setTargetPosition(pos[2]);
        right_rear_drive.setTargetPosition(pos[3]);
    }

    public Pose2D getPinpointPosition(){
        return PinPoint.getPosition();
    }
    public void PinPointUpdate(){
        PinPoint.update();
    }
}

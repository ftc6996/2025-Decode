package org.firstinspires.ftc.teamcode6996_demi;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="DriverController", group="TeleOp")
public class DriverController extends OpMode{

    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 3.61*5.23;
    static final double     WHEEL_DIAMETER_INCHES   = 104/25.4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    /* Declare OpMode members. */
    public DcMotor      left_front_drive  = null;
    public DcMotor      left_rear_drive   = null;
    public DcMotor      right_front_drive = null;
    public DcMotor      right_rear_drive  = null;
    public IMU imu;
    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED = 0.2;
    static final double SPEED_INCREMENT = 0.1;
    public double current_speed = .5;
    
    static final int    DRIVER_MODE_FIELD = 0;
    static final int    DRIVER_MODE_ROBOT = 1;
    public int driver_mode = DRIVER_MODE_ROBOT;
    public String driver_mode_string = "Robot centric";
    public Gamepad saved_gamepad1 = new Gamepad();
    public Gamepad saved_gamepad2 = new Gamepad();
    
    private ElapsedTime runtime = new ElapsedTime();
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection  IMU_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    public RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(IMU_LOGO_DIRECTION, IMU_FACING_DIRECTION);
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        init_chassis();
        
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }

    private void init_chassis()
    {
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");
        
        left_front_drive.setDirection(DcMotor.Direction.FORWARD);
        left_rear_drive.setDirection(DcMotor.Direction.FORWARD);
        right_front_drive.setDirection(DcMotor.Direction.REVERSE);
        right_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Mecanum drive is controlled with three axes: 
        //  drive (front-and-back),
        //  strafe (left-and-right), and 
        //  twist (rotating the whole chassis).
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        boolean strafe_left = gamepad1.left_bumper;
        boolean strafe_right = gamepad1.right_bumper;
        
        //driver controls
        if (gamepad1.options)
        {
            imu.resetYaw();
        }
        if (gamepad1.back && !saved_gamepad1.back)
        {
            if (driver_mode == DRIVER_MODE_ROBOT)
            {
                driver_mode = DRIVER_MODE_FIELD;
                driver_mode_string = "Field centric";
            }
            else
            {
                driver_mode = DRIVER_MODE_ROBOT;
                driver_mode_string = "Robot centric";
            }
        }


        if (gamepad1.dpad_up && !saved_gamepad1.dpad_up)
        {
            current_speed += SPEED_INCREMENT;
            if (current_speed > MAX_MOVE_SPEED)
            {
                current_speed = MAX_MOVE_SPEED;
            }
            //SetLED(LED_STATUS_OFF);
        }
        else if (gamepad1.dpad_down && !saved_gamepad1.dpad_down)
        {
            current_speed -= SPEED_INCREMENT;
            if (current_speed < MIN_MOVE_SPEED)
            {
                current_speed = MIN_MOVE_SPEED;
            }
            //SetLED(LED_STATUS_OFF);
        }

        if (gamepad1.dpad_left)
        {
            twist = .25;
        }
        else if (gamepad1.dpad_right)
        {
           twist = -.25;
        }
        
        if (strafe_left)
        {
            strafe = -current_speed;
        }
        else if (strafe_right)
        {
            strafe = current_speed;
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
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading_deg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double new_strafe = strafe * Math.cos(heading) - drive * Math.sin(heading);
        double new_drive  = strafe * Math.sin(heading) + drive * Math.cos(heading); 
        if (DRIVER_MODE_FIELD == driver_mode)
        {
            drive = -new_drive;
            strafe = -new_strafe;
        }
        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
            (drive + strafe + twist),
            (drive - strafe - twist),
            (drive - strafe + twist),
            (drive + strafe - twist)
        };

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) {
                max = Math.abs(speed);
            }
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        for(int i=0; i <speeds.length; i++)
        {
            if (max > 1)
            {
                speeds[i] /= max;
            }
            speeds[i] *= current_speed;
        }
        
        // apply the calculated values to the motors.
        move(speeds[0], speeds[1], speeds[2], speeds[3]);

        telemetry.addData("Speed%: ", current_speed);
        telemetry.addData("heading (degrees)", heading_deg);
        telemetry.addData("Driver Mode", driver_mode_string);
        telemetry.update();
        
        //save all of the gamepad 
        saved_gamepad1.copy(gamepad1);
        saved_gamepad2.copy(gamepad2);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        stop_all_move();
    }
    
    public void stop_all_move()
    {
        move(0,0,0,0);
    }
    public void move(double left_front, double right_front, double left_back, double right_back)
    {
        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);
        
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back);  
    }
}

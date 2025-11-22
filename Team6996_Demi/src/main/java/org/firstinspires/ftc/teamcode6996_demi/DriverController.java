package org.firstinspires.ftc.teamcode6996_demi;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="DriverController", group="TeleOp")
public class DriverController extends OpMode{

    private MecanumDrive robot;

    private  EndgameController endgameControl;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    public DcMotor turret_Motor;
    public Servo hood_Servo;


    static final double     COUNTS_PER_MOTOR_REV    = 28;
    static final double     DRIVE_GEAR_REDUCTION    = 3.61*5.23;
    static final double     WHEEL_DIAMETER_INCHES   = 104/25.4;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

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

    boolean lastRightTriggerPressed = false;
    boolean rightTriggerPressed = false;

    private ElapsedTime runtime = new ElapsedTime();

    private static final int TAG_ANY = -1;
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL = 24;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = TAG_ANY;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal  ;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private enum IntakeStatus {
        IDLE,
        RUNNING,
        OPPOSITE;
    }
    private IntakeStatus intake_status = IntakeStatus.IDLE;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        robot = new MecanumDrive();
        robot.init(hardwareMap);

        endgameControl = new EndgameController();
        endgameControl.init(hardwareMap, telemetry);
        initAprilTag();
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("Version", "4");
        telemetry.update();
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
        robot.PinPointUpdate();
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        boolean strafe_left = gamepad1.left_bumper;
        boolean strafe_right = gamepad1.right_bumper;

        boolean endgameStart = false;

        lookingForTag();

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }

        //driver controls
        if (gamepad1.options)
        {
            robot.stop();
            robot.imu.resetYaw();
            robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        if (gamepad1.bWasPressed())
        {
            robot.intake(1);
            intake_status = IntakeStatus.RUNNING;
        }
        else if ( gamepad1.xWasPressed())
        {
            robot.intake(-1);
            intake_status = IntakeStatus.OPPOSITE;
        }
        else if (gamepad1.aWasPressed())
        {
            robot.intake(0);
            intake_status = IntakeStatus.IDLE;
        }

        if (gamepad1.yWasPressed()){
            turret_Motor.setPower(.8);
        }
        if (gamepad1.aWasPressed()){
            hood_Servo.setPosition(.7);
        }

        rightTriggerPressed = gamepad1.right_trigger > 0.5;

        if (rightTriggerPressed && !lastRightTriggerPressed) {
            endgameStart = true;
        } else {
            endgameStart = false;
        }

        lastRightTriggerPressed = rightTriggerPressed;

        if (gamepad1.dpad_up && gamepad1.dpadUpWasPressed()) {
            endgameControl.adjustDriveSpeed(0.1);
        } else if (gamepad1.dpad_down && gamepad1.dpadDownWasPressed()) {
            endgameControl.adjustDriveSpeed(-0.1);
        }

        if (gamepad1.y && gamepad1.yWasPressed()) {
            endgameControl.switchAllianceColor();
        }

            /*
        if (strafe_left)
        {
            strafe = -current_speed;
        }
        else if (strafe_right)
        {
            strafe = current_speed;
        }
*/
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
        double heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading_deg = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double new_strafe = strafe * Math.cos(heading) - drive * Math.sin(heading);
        double new_drive  = strafe * Math.sin(heading) + drive * Math.cos(heading); 
        if (DRIVER_MODE_FIELD == driver_mode)
        {
            drive = -new_drive;
            strafe = -new_strafe;
        }

        // If  is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (strafe_left && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            twist   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, twist);
        }
        if (!endgameControl.processUpdate(robot, endgameStart)) {
            robot.move(drive, strafe, twist);
        }

        endgameControl.showData();

        telemetry.addData("Speed%: ", current_speed);
        telemetry.addData("heading (degrees)", heading_deg);
        telemetry.addData("Left Joy X", gamepad1.left_stick_x);
        telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
        telemetry.addData("Driver Mode", driver_mode_string);
        int [] target = robot.getAllPositions();
        telemetry.addData("Path", "Driving");
        telemetry.addData("LF", target[0]);
        telemetry.addData("RF", target[1]);
        telemetry.addData("LB", target[2]);
        telemetry.addData("RB", target[3]);
        telemetry.addData("Intake", intake_status);
        telemetry.addData("positionX", Math.round(robot.getPinpointPosition().getX(DistanceUnit.MM)));
        telemetry.addData("positionY", Math.round(robot.getPinpointPosition().getY(DistanceUnit.MM)));
        telemetry.addData("heading",Math.round(robot.getPinpointPosition().getHeading(AngleUnit.DEGREES)));
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
        robot.setRawPower(0,0,0,0);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void mySleep(int milliseconds)
    {
        ElapsedTime timer = new ElapsedTime();
        int x = 0;
        while(timer.milliseconds() < milliseconds)
        {
            x++;
        }
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        runtime.startTime();
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                mySleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                mySleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            mySleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            mySleep(20);
    }

    private void lookingForTag()
    {
        targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
}

package org.firstinspires.ftc.teamcode;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading
import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Drive.*;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="DriverController", group="TeleOp")
public class DriverController extends OpMode{

    public Robot robot = new Robot();
    public double current_speed = .5;
    

    public int driver_mode = DRIVER_MODE_ROBOT;
    public String driver_mode_string = "Robot centric";
    public Gamepad saved_gamepad1 = new Gamepad();
    public Gamepad saved_gamepad2 = new Gamepad();
    
    private ElapsedTime runtime = new ElapsedTime();
    private boolean targetFound = false;
      /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        robot.init(hardwareMap);
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("Version", "1");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        robot.update();
        robot.process(runtime);
        // Mecanum drive is controlled with three axes: 
        //  drive (front-and-back),
        //  strafe (left-and-right), and 
        //  twist (rotating the whole chassis).
        double drive  = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist  = gamepad1.right_stick_x;

        boolean strafe_left = gamepad1.left_bumper;
        boolean strafe_right = gamepad1.right_bumper;

        //lookingForTag();

        // Tell the driver what we see, and what to do.
        /*
        if (targetFound) {
            telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Drive using joysticks to find valid target\n");
        }
*/
        //driver controls
        if (gamepad1.optionsWasPressed())
        {
            robot.DriveTrain().stop();
            robot.DriveTrain().resetYaw();
            robot.DriveTrain().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveTrain().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad1.backWasPressed())
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

        if (gamepad1.dpadUpWasPressed())
        {
            current_speed += SPEED_INCREMENT;
            if (current_speed > MAX_MOVE_SPEED)
            {
                current_speed = MAX_MOVE_SPEED;
            }
            //SetLED(LED_STATUS_OFF);
        }
        else if (gamepad1.dpadDownWasPressed())
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
        double heading = robot.DriveTrain().getOrientation().getYaw(AngleUnit.RADIANS);
        double heading_deg = robot.DriveTrain().getOrientation().getYaw(AngleUnit.DEGREES);
        double new_strafe = strafe * Math.cos(heading) - drive * Math.sin(heading);
        double new_drive  = strafe * Math.sin(heading) + drive * Math.cos(heading); 
        if (DRIVER_MODE_FIELD == driver_mode)
        {
            drive = -new_drive;
            strafe = -new_strafe;
        }
/*
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

 */
        robot.move(drive, strafe, twist);

        robot.processTelemetry(telemetry);
        telemetry.addData("Speed%: ", current_speed);
        telemetry.addData("heading (degrees)", heading_deg);
        telemetry.addData("Left Joy X", gamepad1.left_stick_x);
        telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
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
        robot.DriveTrain().stop();
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
    */

}

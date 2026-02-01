package org.firstinspires.ftc.teamcode;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading
import static org.firstinspires.ftc.teamcode.Constants.Drive.*;
import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.*;
import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;


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
    private boolean intake_on = false;
    private boolean outtake_on = false;
    private int alliance = kNOT_SET;
    private int bonusVelocity = 0;
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
        robot.getAprilTag();
        if (gamepad1.bWasPressed() || gamepad2.bWasPressed())
        {
            alliance = kALLIANCE_RED;
            robot.setAlliance(alliance);
        }
        if (gamepad1.xWasPressed() || gamepad2.xWasPressed())
        {
            alliance = kALLIANCE_BLUE;
            robot.setAlliance(alliance);
        }

        if (alliance == kALLIANCE_RED)
            telemetry.addData("Alliance", "kALLIANCE_RED");
        else if (alliance == kALLIANCE_BLUE)
            telemetry.addData("Alliance", "kALLIANCE_BLUE");
        else
            telemetry.addData("Alliance", "kNOT_SET");

        telemetry.addData("Pipeline", robot.launcher.limeLight.getPipeline());
        telemetry.addData("Version", "4");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();
        robot.launcher.limeLight.vision.start();
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
        double drive  = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double twist  = -gamepad1.right_stick_x;

        boolean strafe_left = gamepad1.left_bumper;
        boolean strafe_right = gamepad1.right_bumper;
        //boolean strafe_left = gamepad1.right_stick_x < 0;
        //boolean strafe_right = gamepad1.right_stick_x > 0;

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

        if (gamepad2.backWasPressed()){
            robot.launcher.rapidFire = true;
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
            strafe = current_speed;
        }
        else if (strafe_right)
        {
            strafe = -current_speed;
        }
        if (gamepad1.aWasPressed())
        {
            if (robot.getKickerState() == Launcher.KickingState.UP)
            {
                robot.setKickerDown();
            }
            else if (robot.getKickerState() == Launcher.KickingState.DOWN)
            {
                robot.setKickerUp();
            }
        }

        if (gamepad1.bWasPressed()) {
            //TODO: start endgame or if it is already started cancel it
        }

        if (gamepad2.dpadLeftWasPressed()){
            robot.launcher.setHoodPosition(kHOOD_MAX_POS);
        }
        if (gamepad2.dpadRightWasPressed()){
            robot.launcher.setHoodPosition(kHOOD_MIN_POS);
        }

        //rotate turret
        if (gamepad2.left_trigger > 0)
        {
            robot.setTurretPower(-gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger > 0)
        {
            robot.setTurretPower(gamepad2.right_trigger);
        }
        else
        {
            robot.setTurretPower(0);
        }

        //aux player wants to auto seek target
        /*if (gamepad2.leftBumperWasPressed())
        {
           robot.seekTagLeft();
        }
        else if (gamepad2.rightBumperWasPressed())
        {
            robot.seekTagRight();
        }*/

        if (gamepad2.dpadUpWasPressed())
        {
            bonusVelocity += 20;
        }else if (gamepad2.dpadDownWasPressed())
        {
            bonusVelocity -= 20;
        }

        if (gamepad2.dpadLeftWasPressed())
        {

        }
        if (gamepad2.dpadRightWasPressed())
        {

        }

        if (gamepad2.rightBumperWasPressed())
        {
            robot.shoot(true, kLAUNCHER_TARGET_VELOCITY_CLOSE, 1);
        }
        if (gamepad2.leftBumperWasPressed())
        {
            robot.shoot(true, kLAUNCHER_TARGET_VELOCITY_FAR + bonusVelocity, 1);
        }

        //allow aux player to turn off shooter
        if (gamepad2.yWasPressed())
        {
            robot.shoot(false, 0, 0);
        }

        //aux player controls intake
        if (gamepad2.aWasPressed())
        {
            intake_on = !intake_on;

            if (intake_on){
                robot.intake(1, false);
            }
            else {
                robot.intake(0, true);
            }
        }
        if (gamepad2.bWasPressed())
        {
            outtake_on = !outtake_on;

            if (outtake_on) {
                robot.intake(-1, false);
            }
            else {
                robot.intake(0, true);
            }
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

        robot.move(drive, strafe, twist);
        robot.processTelemetry(telemetry);
        telemetry.addData("Speed%: ", current_speed);
        telemetry.addData("Bonus Vel ", bonusVelocity);
        //telemetry.addData("heading (degrees)", heading_deg);
        //telemetry.addData("Left Joy X", gamepad1.left_stick_x);
        //telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
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

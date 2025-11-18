package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Constants.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.opencv.core.Mat;

@TeleOp(name="TeleOpMiniBot", group="TeleOp")
public class TeleOpMiniBot extends OpMode {
    private Robot robot;
    double drive, strafe, twist = 0;
    boolean strafe_left, strafe_right = false;
    double current_speed, intake_speed = 0;
    boolean enable_moving = false;
    int alliance = kNOT_SET;
    String allianceString = "NOT SET";
    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap);
    }

    public void setAlliance(int a)
    {
        alliance = a;
        if (a == kALLIANCE_RED)
        {
            allianceString = "RED";

        } else if (a == kALLIANCE_BLUE)
        {
            allianceString = "BLUE";
        }
        robot.setAlliance(alliance);
    }

    @Override
    public void init_loop() {

        if (gamepad1.x || gamepad2.x) {
            setAlliance(kALLIANCE_BLUE);
        }
        if (gamepad1.b || gamepad2.b) {
            setAlliance(kALLIANCE_RED);
        }

        telemetry.addData("Version", "7");
        telemetry.addData("Alliance", allianceString);
        telemetry.update();
    }
    @Override
    public void start() {
        current_speed = robot.getMaxSpeed();
    }

    @Override
    public void loop() {
        processJoystick();
        processTracking();
        if (enable_moving) {
            robot.move(drive, strafe, twist);

            robot.intake(intake_speed);
        }

        //robot.processTelemetry(telemetry);
        //TODO: push some of this down to robot
        telemetry.addData("Move Enabled: ", enable_moving);
        telemetry.addData("Speed% ", current_speed);
        telemetry.addData("Drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("twist: ", twist);
        telemetry.update();
    }

    public double deadband(double value, double tolerance)
    {
        double new_value = 0;
        if (Math.abs(value) > tolerance)
        {
            new_value = value;
        }
        return new_value;
    }
    public void processJoystick()
    {
        drive  = deadband(gamepad1.left_stick_y, 0.1);
        strafe = -deadband(gamepad1.left_stick_x, 0.1);
        twist  = -deadband(gamepad1.right_stick_x, 0.1);
        strafe_left = gamepad1.left_bumper;
        strafe_right = gamepad1.right_bumper;

        if (gamepad1.dpadUpWasPressed())
        {
            robot.changeSpeed(Drive.SPEED_INCREMENT);
            current_speed = robot.getMaxSpeed();
        }
        else if (gamepad1.dpadDownWasPressed())
        {
            robot.changeSpeed(-Drive.SPEED_INCREMENT);
            current_speed = robot.getMaxSpeed();
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
        if (gamepad1.backWasPressed())
        {
            enable_moving = !enable_moving;
        }

        //temporary handling for intake
        if (gamepad1.right_trigger > 0)
        {
            intake_speed = 1;// gamepad1.right_trigger;
        }
        else if (gamepad1.left_trigger > 0)
        {
            intake_speed = -1;//gamepad1.left_trigger;
        }
        else if (gamepad1.xWasPressed())
        {
            intake_speed = 0;
        }
    }

    public void processTracking()
    {
        //process limelight
        //move turrent, hood
        //are we in range?
        //are we out of range?
    }
}

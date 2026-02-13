package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kLAUNCHER_TARGET_VELOCITY_CLOSE;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kLAUNCHER_TARGET_VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.Constants.RGB_SERVO_LIGHT.*;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.Blinky;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@TeleOp(name="TestTurret", group="TEST")
//@Disabled
public class TestTurret extends OpMode {
    double position = 0;
    Launcher launcher;
    Blinky blinky;
    private Servo rgb_light;
    private GoBildaPinpointDriver pinpoint;

    private int alliance = kALLIANCE_RED;

    public double highVelocity = 1500;
    public double lowVelocity = 800;
    public double currentTargetVelocity = highVelocity;
    public double F = 12;
    public double P = 94;
    public double[] stepSize = {10.0, 1.0, 0.1, 0.001, 0.0001};
    public int stepSizeIndex = 1;
    public double tolerance = 2;
    private void initPinPoint()
    {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0));
        pinpoint.setOffsets(90,0,DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap);
        launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_RED);
        launcher.target_tag = kTAG_GOAL_RED;

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0,0, F);
        launcher.turret_flywheel_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        blinky = new Blinky();
        blinky.init(hardwareMap);
        blinky.setUnknownAlliance();
        rgb_light = hardwareMap.get(Servo.class, "rgb_light");
        rgb_light.setPosition(kBLACK);

        initPinPoint();
    }

    @Override
    public void init_loop() {

        if (gamepad1.bWasPressed())
        {
            alliance = kALLIANCE_RED;
            launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_RED);
            launcher.target_tag = kTAG_GOAL_RED;
            rgb_light.setPosition(kRED);
        }
        if (gamepad1.xWasPressed())
        {
            alliance = kALLIANCE_BLUE;
            launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_BLUE);
            launcher.target_tag = kTAG_GOAL_BLUE;
            rgb_light.setPosition(kBLUE);
        }

        if (alliance == kALLIANCE_RED)
            telemetry.addData("Alliance", "kALLIANCE_RED");
        else if (alliance == kALLIANCE_BLUE)
            telemetry.addData("Alliance", "kALLIANCE_BLUE");
        telemetry.addData("Version", "4");
        telemetry.update();
    }

    @Override
    public void start()
    {

    }
    @Override
    public void loop() {

        pinpoint.update();
        launcher.process(pinpoint.getPosition());

        if (launcher.isTargetFound)
        {
            blinky.setInRange();
        }
        else {
            blinky.setUnknownAlliance();
        }
        if (Math.abs(gamepad1.left_stick_x) > .15)
        {
            launcher.setTurretPower(gamepad1.left_stick_x);
            launcher.turningState = Launcher.TurningState.IDLE;
        }
        else
        {
            //only if we are in IDLE mode (i.e. manual mode) that we want to stop the turret
            if (launcher.turningState == Launcher.TurningState.IDLE)
            {
                launcher.setTurretPower(0);
            }
        }

        if (gamepad1.leftTriggerWasPressed())
        {
            launcher.turningState = Launcher.TurningState.START_LEFT;
        }
        else if (gamepad1.rightTriggerWasPressed())
        {
            launcher.turningState = Launcher.TurningState.START_RIGHT;
        }


        if (gamepad1.aWasPressed())
        {
            launcher.setHoodPosition(kHOOD_MAX_POS);
            telemetry.addData("Hood Position",".5");
        }
        else if (gamepad1.bWasPressed())
        {
            launcher.setHoodPosition(kHOOD_MIN_POS);
            telemetry.addData("Hood Position","1");
        }

        if (gamepad2.backWasPressed()){
            launcher.rapidFire = true;
        }

        if (gamepad1.dpadUpWasPressed())
        {
            position += 0.05;
        }else if (gamepad1.dpadDownWasPressed())
        {
            position -= 0.05;
        }

        if (gamepad1.rightBumperWasPressed())
        {
            launcher.shoot(true, kLAUNCHER_TARGET_VELOCITY_CLOSE, 1);
        }
        if (gamepad1.leftBumperWasPressed())
        {
            launcher.shoot(true, kLAUNCHER_TARGET_VELOCITY_FAR, 1);
        }

       // launcher.turret_feeder_servo.setPosition(position);
        //telemetry.addData("kicker position", position);
/*
        //switch between high and low velocity
        if (gamepad2.yWasPressed())
        {
            if (currentTargetVelocity == highVelocity)
                currentTargetVelocity = lowVelocity;
            else currentTargetVelocity = highVelocity;
        }

        //step through the step sizes
        if (gamepad2.bWasPressed())
        {
            stepSizeIndex = (stepSizeIndex + 1) % stepSize.length;
        }

        if (gamepad2.dpadLeftWasPressed())
        {
            F -= stepSize[stepSizeIndex];
        }
        if (gamepad2.dpadRightWasPressed())
        {
            F += stepSize[stepSizeIndex];
        }

        if (gamepad2.dpadUpWasPressed())
        {
            P += stepSize[stepSizeIndex];
        }
        if (gamepad2.dpadDownWasPressed())
        {
            P -= stepSize[stepSizeIndex];
        }

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0,0, F);
        launcher.turret_flywheel_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        launcher.turret_flywheel_motor.setVelocity(currentTargetVelocity);

        double currentVelocity = launcher.turret_flywheel_motor.getVelocity();
        double error = currentTargetVelocity - currentVelocity;

        telemetry.addData(" Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("           Error", "%.2f", error);
        telemetry.addLine("------------------------------------");
        telemetry.addData(" Tuning P", "%.4f (U/D)", P);
        telemetry.addData(" Tuning F", "%.4f (L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSize[stepSizeIndex]);

*/
        if (alliance == kALLIANCE_RED)
            telemetry.addData("Alliance", "kALLIANCE_RED");
        else if (alliance == kALLIANCE_BLUE)
            telemetry.addData("Alliance", "kALLIANCE_BLUE");
        telemetry.addLine("-----------------------------------");
        telemetry.addData("Hood Pos",launcher.getHoodPositon());
        telemetry.addData("Left Mag",launcher.isLeftSensorTriggered());
        telemetry.addData("Right Mag",launcher.isRightSensorTriggered());
        telemetry.addData("Launch Pos",launcher.getPositon());
        telemetry.addLine("-----------------------------------");
        telemetry.addData("Pipeline",launcher.limeLight.getPipeline());
        telemetry.addData("turret state",launcher.turningState);

        telemetry.addData("rapidFire", launcher.rapidFire);

        telemetry.addData("Target Found",launcher.isTargetFound);
        if (launcher.isTargetFound) {
            telemetry.addData("ID seen", launcher.limeLight.getTagID());
            telemetry.addData("Distance", launcher.limeLight.getTagDistance());
            telemetry.addData("Lime  Pose", launcher.limeLight.tagPose.getPosition().toUnit(DistanceUnit.INCH).toString());
            telemetry.addData("dX, dY", "%.2f, %.2f", launcher.dX, launcher.dY);

        }
        telemetry.update();
    }
}

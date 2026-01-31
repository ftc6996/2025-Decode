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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.Blinky;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@TeleOp(name="TestTurret", group="TEST")
@Disabled
public class TestTurret extends OpMode {
    double position = 0;
    Launcher launcher;
    Blinky blinky;
    private Servo rgb_light;
    private GoBildaPinpointDriver pinpoint;

    private int alliance = kALLIANCE_RED;

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

        blinky = new Blinky();
        blinky.init(hardwareMap);
        blinky.setUnknownAlliance();
        rgb_light = hardwareMap.get(Servo.class, "rgb_light");
        rgb_light.setPosition(kBLACK);
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
        if (gamepad1.left_trigger > 0)
        {
            launcher.setTurretPower(gamepad1.left_trigger);
            launcher.turningState = Launcher.TurningState.IDLE;
        }
        else if (gamepad1.right_trigger > 0)
        {
            launcher.setTurretPower(-gamepad1.right_trigger);
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
        if (gamepad1.leftBumperWasPressed())
        {
            launcher.turningState = Launcher.TurningState.START_LEFT;
        }
        else if (gamepad1.rightBumperWasPressed())
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

        if (gamepad2.rightBumperWasPressed())
        {
            launcher.shoot(true, kLAUNCHER_TARGET_VELOCITY_CLOSE, 1);
        }
        if (gamepad2.leftBumperWasPressed())
        {
            launcher.shoot(true, kLAUNCHER_TARGET_VELOCITY_FAR, 1);
        }

       // launcher.turret_feeder_servo.setPosition(position);
        //telemetry.addData("kicker position", position);

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
            telemetry.addData("Pose", launcher.limeLight.tagPose.getPosition().toUnit(DistanceUnit.INCH).toString());
        }
        telemetry.update();
    }
}

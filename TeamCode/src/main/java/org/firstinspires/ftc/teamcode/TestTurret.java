package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.Constants.*;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Blinky;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@TeleOp(name="TestTurret", group="TEST")
//@Disabled
public class TestTurret extends OpMode {

    Launcher launcher;
    Blinky blinky;

    private int alliance = kALLIANCE_RED;
    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap);
        launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_RED);
        launcher.target_tag = kTAG_GOAL_RED;

        blinky = new Blinky();
        blinky.init(hardwareMap);
        blinky.setUnknownAlliance();
    }

    @Override
    public void init_loop() {

        if (gamepad1.bWasPressed())
        {
            alliance = kALLIANCE_RED;
            launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_RED);
            launcher.target_tag = kTAG_GOAL_RED;
        }
        if (gamepad1.xWasPressed())
        {
            alliance = kALLIANCE_BLUE;
            launcher.limeLight.setPipeline(kPIPELINE_ALLIANCE_BLUE);
            launcher.target_tag = kTAG_GOAL_BLUE;
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

        launcher.process();
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

        telemetry.addData("Target Found",launcher.isTargetFound);
        if (launcher.isTargetFound) {
            telemetry.addData("ID seen", launcher.limeLight.getTagID());
            telemetry.addData("Distance", launcher.limeLight.getTagDistance());
            telemetry.addData("Pose", launcher.limeLight.tagPose.getPosition().toUnit(DistanceUnit.INCH).toString());
        }
        telemetry.update();
    }
}

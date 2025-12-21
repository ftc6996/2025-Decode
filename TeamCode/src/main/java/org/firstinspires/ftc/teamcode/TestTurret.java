package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MIN_POS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;

@TeleOp(name="TestTurret", group="TEST")
@Disabled
public class TestTurret extends OpMode {

    Launcher launcher;

    @Override
    public void init() {
        launcher = new Launcher();
        launcher.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Version", "4");
        telemetry.update();
    }

    @Override
    public void loop() {

        launcher.process();

        if (gamepad1.left_trigger > 0)
        {
            launcher.setTurretPower(gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > 0)
        {
            launcher.setTurretPower(-gamepad1.right_trigger);
        }
        else
        {
            launcher.setTurretPower(0);
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

        telemetry.addData("Position",launcher.getHoodPositon());
        telemetry.addData("Left Mag",launcher.isLeftSensorTriggered());
        telemetry.addData("Right Mag",launcher.isRightSensorTriggered());
        telemetry.addData("Version", "5");
        telemetry.update();
    }
}

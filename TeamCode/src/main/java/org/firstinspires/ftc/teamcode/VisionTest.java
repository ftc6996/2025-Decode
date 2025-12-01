package org.firstinspires.ftc.teamcode6996_demi;

import static org.firstinspires.ftc.teamcode.Constants.kALLIANCE_BLUE;
import static org.firstinspires.ftc.teamcode.Constants.kALLIANCE_RED;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.LimeLight;


@TeleOp(name="VisionTest", group="TeleOp")
public class VisionTest extends OpMode {

    Limelight3A vision;
    public LimeLight limeLight;
private String allianceString = "none";
    @Override
    public void init() {
        vision = hardwareMap.get(Limelight3A.class, "limelight");
        limeLight = new LimeLight(vision);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Version", "2");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (gamepad1.x || gamepad2.x) {
            limeLight.setPipeline(1);
            allianceString = "blue";
        }
        if (gamepad1.b || gamepad2.b) {
            limeLight.setPipeline(2);
            allianceString = "red";
        }

        telemetry.addData("Alliance", allianceString);
        limeLight.getAprilTags();
        telemetry.addData("tag id", limeLight.getTagID());
        telemetry.addData("tag location x", limeLight.getTagLocationX());
        telemetry.addData("tag location y", limeLight.getTagLocationY());
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.LimeLight;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@TeleOp(name="VisionTest", group="TeleOp")
public class VisionTest extends OpMode {

    Limelight3A vision;
    public LimeLight limeLight;

    @Override
    public void init() {
        vision = hardwareMap.get(Limelight3A.class, "Limelight3A");
        limeLight = new LimeLight(vision);
        limeLight.setPipeline(0);
    }

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

    }
    @Override
    public void loop() {
        limeLight.getAprilTags();

    }
}

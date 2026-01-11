package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Blinky;

@TeleOp(name="TestBlinky", group="TEST")
//@Disabled
public class TestBlinky extends OpMode {

    private Blinky blinky;
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        blinky = new Blinky();
        blinky.init(hardwareMap);
        blinky.setUnknownAlliance();
    }

    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void init_loop() {
        telemetry.addData("Version", "1");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Version", "1");
        telemetry.addData("Runtime", runtime.seconds());
        blinky.process(runtime);
    }
}

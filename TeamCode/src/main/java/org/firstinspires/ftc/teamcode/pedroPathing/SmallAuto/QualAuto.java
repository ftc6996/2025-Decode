package org.firstinspires.ftc.teamcode.pedroPathing.SmallAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name="QualAuto", group="TEST")
public class QualAuto extends OpMode {
    private Robot r;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void init() {
        r = new Robot();
        r.init(hardwareMap);

    }

    @Override
    public void init_loop() {
        telemetry.addData("Version", "4");
        telemetry.update();
    }

    @Override
    public void start(){
        runtime.reset();
        r.move(1,0,0);
    }
    @Override
    public void loop() {

        if (runtime.seconds() > 2)
        {
           r.DriveTrain().stop();
        }
    }


}

package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "TestMotors")
public class TestMotors extends OpMode {
    private MecanumDrive robot;

    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //robot.setRawPower(1);
        robot.move(1,0,0);
        int[] a=robot.getAllPositions();
        telemetry.addLine(a.toString());
        telemetry.update();
    }
}
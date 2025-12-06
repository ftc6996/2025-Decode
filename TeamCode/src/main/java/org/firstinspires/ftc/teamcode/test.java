package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "TEST")
public class test extends OpMode{
    private Robot robot;
    @Override
    public void init() {
        // Hardware map setup
        robot = new Robot();
        robot.init(hardwareMap);
    }
    @Override
    public void loop() {
        robot.update();
        //robot.getPinpointPosition();
        if(robot.mecanumDrive.pinpoint != null){
            double PinPointx = robot.mecanumDrive.pinpoint.getPosition().getX(DistanceUnit.MM);
            double PinPointy = robot.mecanumDrive.pinpoint.getPosition().getY(DistanceUnit.MM);
            double currentHeading = robot.mecanumDrive.pinpoint.getHeading(AngleUnit.DEGREES);
            telemetry.addData("PinPointx in MM", PinPointx);
            telemetry.addData("PinPointy in MM", PinPointy);
            telemetry.addData("currentHeading in DEG", Math.toDegrees(currentHeading));
        }else{
            telemetry.addData("Pinpoint","not initialised");
        }
        telemetry.update();
    }
}
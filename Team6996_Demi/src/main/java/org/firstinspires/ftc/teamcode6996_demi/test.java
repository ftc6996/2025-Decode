package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "TEST")
public class test extends OpMode{
    private MecanumDrive robot;
    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
    }
    public static double angleError(double target, double curent){
        double error = target - curent;
        double errorr = (error + 180) % 360;
        if(errorr < 0){
            errorr = errorr + 360;
        }
        return errorr - 180;
    }
    @Override
    public void loop() {
        robot.PinPointUpdate();
        //robot.getPinpointPosition();
        //if(robot. != null){
            double stopTurnRangeDeg = 70;//deg
            double targetHeadingDeg = 190;// angle that the robot ends at in deg
            double startAt = 0;

            //if (targetHeadingDeg < 0){
           //     targetHeadingDeg = -360-targetHeadingDeg;
            //}
            double PinPointx = robot.getPinpointPosition().getX(DistanceUnit.MM);
            double PinPointy = robot.getPinpointPosition().getY(DistanceUnit.MM);
            double currentHeadingDeg = startAt-(robot.getPinpointPosition().getHeading(AngleUnit.DEGREES));
            double moveDeg = angleError(targetHeadingDeg,currentHeadingDeg);
            double thing = targetHeadingDeg -currentHeadingDeg;
            telemetry.addData("PinPointx in MM", PinPointx);
            telemetry.addData("PinPointy in MM", PinPointy);
            telemetry.addData("currentHeading in DEG", currentHeadingDeg);
            telemetry.addData("targetHeadingDeg", targetHeadingDeg);
            telemetry.addData("moveDeg", moveDeg);
            telemetry.addData("thing", thing);

            robot.move(0,0,moveDeg);

            if ((Math.abs(targetHeadingDeg) - Math.abs(currentHeadingDeg))<= stopTurnRangeDeg){
                telemetry.addLine("STOPED");
                robot.move(0,0,0);
            }
        //}else{
            //telemetry.addData("Pinpoint","not initialised");
        //}
            telemetry.update();
    }
}
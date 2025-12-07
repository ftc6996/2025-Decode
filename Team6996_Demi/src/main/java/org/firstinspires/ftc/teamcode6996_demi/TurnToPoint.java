package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "TurnToPoint")
public class TurnToPoint extends OpMode{
    private MecanumDrive robot;
    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
    }
    public static double angleError(double target, double current){
        double error = target - current;
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
        double slowSpeed = 0.01;
        double stopTurnRangeDeg = 0;//deg
        double targetHeadingDeg = 180;// angle that the robot ends at in deg
        //if (targetHeadingDeg < 0){
       //     targetHeadingDeg = -360-targetHeadingDeg;
        //}
        double currentHeadingDeg = robot.getPinpointPosition().getHeading(AngleUnit.DEGREES);
        double moveDeg = angleError(targetHeadingDeg,currentHeadingDeg);
        double thing = targetHeadingDeg -currentHeadingDeg;
        double moveDegAbs = 0;

        telemetry.addData("currentHeading in DEG", currentHeadingDeg);
        telemetry.addData("targetHeadingDeg", targetHeadingDeg);
        telemetry.addData("moveDeg", moveDeg);
        telemetry.addData("thing", thing);

        //slowCalculator{
        //Maybe later
        //}

        if ((Math.abs(targetHeadingDeg) - Math.abs(currentHeadingDeg))<= stopTurnRangeDeg){
            telemetry.addLine("STOPED TurnRange");
            moveDeg = 0;
        }else{
            //TranslatorTurn{
            moveDegAbs = Math.abs(moveDeg*slowSpeed);
            moveDeg = moveDeg*slowSpeed;
            if(moveDegAbs <= 0.1) {
                moveDeg = 0;
                telemetry.addLine("STOPED slowSpeed");
            }
            telemetry.addData("moveDeg",moveDeg);
            //}
        }
    //}else{
        //telemetry.addData("Pinpoint","not initialised");
    //}
        telemetry.update();
        robot.move(0,0,moveDeg);
    }
}
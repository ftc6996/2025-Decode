package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "GoToPoint")
public class GoToPoint extends OpMode{
    private MecanumDrive robot;
    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
    }
    public static double angleError(double target, double current){
        double error = target - current;
        error = (error + 180) % 360;
        if(error < 0){
            error = error + 360;
        }
        return error - 180;
    }
    @Override
    public void loop() {

        robot.PinPointUpdate();
        //robot.getPinpointPosition();
        //if(robot. != null){
        double slowSpeedMove = 0.0025;
        double slowSpeedTurn = 0.01;
        double stopRangeMm = 5;//Mm
        double stopTurnRangeDeg = 0;//deg
        double targetXMm = 100;// Distance that the robot ends at in Mm
        double targetYMm = 1000;// Distance that the robot ends at in Mm
        double targetHeadingDeg = 180;// angle that the robot ends at in deg
        targetXMm = -targetXMm;
        //if (targetHeadingDeg < 0){
        //     targetHeadingDeg = -360-targetHeadingDeg;
        //}
        double currentXMm = robot.getPinpointPosition().getX(DistanceUnit.MM);
        double currentYMm = robot.getPinpointPosition().getY(DistanceUnit.MM);
        double currentHeadingDeg = robot.getPinpointPosition().getHeading(AngleUnit.DEGREES);
        double XMm = targetXMm - currentXMm;
        double YMm = targetYMm - currentYMm;
        double moveDeg = angleError(targetHeadingDeg,currentHeadingDeg);
        double XMmAbs = 0;
        double YMmAbs = 0;
        double moveDegAbs = 0;

        telemetry.addData("currentXMm", currentXMm);
        telemetry.addData("currentYMm", currentYMm);
        telemetry.addData("targetXMM", targetXMm);
        telemetry.addData("targetYMM", targetXMm);
        //telemetry.addData("thing", thing);

        //slowCalculator{
        //Maybe later
        //}

        if ((Math.abs(targetXMm) - Math.abs(currentXMm))<=stopRangeMm){
            telemetry.addLine("XSTOPED");
            XMm = 0;
        }else{
            //TranslatorX{
            XMmAbs = Math.abs(XMm*slowSpeedMove);
            XMm = XMm*slowSpeedMove;
            if(XMmAbs <= 0.1) {
                XMm = 0;
            }
            telemetry.addData("XMm",XMm);
            //}
        }
        if ((Math.abs(targetYMm) - Math.abs(currentYMm))<=stopRangeMm){
            telemetry.addLine("YSTOPED");
            YMm = 0;
        }else{
            //TranslatorY{
            YMmAbs = Math.abs(YMm*slowSpeedMove);
            YMm = YMm*slowSpeedMove;
            if(YMmAbs <= 0.1) {
                YMm = 0;
            }
            telemetry.addData("YMm",YMm);
            //}
        }
        if ((Math.abs(targetHeadingDeg) - Math.abs(currentHeadingDeg))<= stopTurnRangeDeg){
            telemetry.addLine("STOPED TurnRange");
            moveDeg = 0;
        }else{
            //TranslatorTurn{
            moveDegAbs = Math.abs(moveDeg*slowSpeedTurn);
            moveDeg = moveDeg*slowSpeedTurn;
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
        robot.move(YMm,XMm,moveDeg);
    }
}
package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "MoveToPoint")
public class MoveToPoint extends OpMode{
    private MecanumDrive robot;
    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
    }
    public static double XError(double target, double current){
        double error = target - current;
        double errorr = (error + 180) % 360;
        if(errorr < 0){
            errorr = errorr 
    @Override
    public void loop() {
        /*
        robot.PinPointUpdate();
        //robot.getPinpointPosition();
        //if(robot. != null){
        double slowSpeed = 0.010;
        double stopRangeMM = 0;//deg
        double targetDistanceMm = 180;// angle that the robot ends at in deg
        double startAt = 0;
        //if (targetHeadingDeg < 0){
       //     targetHeadingDeg = -360-targetHeadingDeg;
        //}
        double PinPointx = robot.getPinpointPosition().getX(DistanceUnit.MM);
        double PinPointy = robot.getPinpointPosition().getY(DistanceUnit.MM);
        //double currentHeadingDeg = startAt-(robot.getPinpointPosition().getHeading(AngleUnit.DEGREES));
        //double moveDeg = angleError(targetDistanceMM,currentHeadingDeg);
        //double thing = targetDistanceMM -currentHeadingDeg;
        //double moveDegAbs = 0;

        telemetry.addData("PinPointx in MM", PinPointx);
        telemetry.addData("PinPointy in MM", PinPointy);
        //telemetry.addData("currentHeading in DEG", currentHeadingDeg);
        telemetry.addData("targetDistanceMM", targetDistanceMm);
        //telemetry.addData("moveDeg", moveDeg);
        //telemetry.addData("thing", thing);

        //slowCalculator{
        //Maybe later
        //}

        //Translator{
        moveMmAbs = Math.abs(moveMm*slowSpeed);
        moveMm = moveMm*slowSpeed;
        if(moveMmAbs <= 0.1) {
            moveMm = 0;
        }
        telemetry.addData("moveMm",moveMm);
        //}

        robot.move(0,0,0);

        if ((Math.abs(targetDistanceMm) - Math.abs(currentDistanceMm))<= stopRangeMm){
            telemetry.addLine("STOPED");
            robot.move(0,0,0);
        }
    //}else{
        //telemetry.addData("Pinpoint","not initialised");
    //}
        telemetry.update();
        */
    }
}
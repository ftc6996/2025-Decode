package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "Auto_DECODE")
public class Auto_DECODE extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive robot;
    private static final int kNOT_SET = -1;
    private static final int kSTART_LOCATION_1 = 0;
    private static final int kSTART_LOCATION_2 = 1;
    private static final int kSTART_LOCATION_3 = 2;
    public int current_start_location = kNOT_SET;

    private static final int kBLUE = 0;
    private static final int kRED = 1;
    public int alliance_color = kNOT_SET;

    @Override
    public void runOpMode() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "4");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        // Run the test auto routine
        runTestAuto();
    }

    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             double timeoutS) {


        // Constants for encoder counts for AndyMark NeveRest 40 (40:1)
        final double COUNTS_PER_MOTOR_REV = 1120; // AM-2964 with 40:1 gearbox
        final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
        final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

        // Calculate target positions
        int [] target = robot.getAllPositions();
        target[0] += (int)(leftFrontInches * COUNTS_PER_INCH);
        target[1] += (int)(rightFrontInches * COUNTS_PER_INCH);
        target[2] += (int)(leftBackInches * COUNTS_PER_INCH);
        target[3] += (int)(rightBackInches * COUNTS_PER_INCH);

        // Set target positions
        robot.setTargetPosition(target);

        // Set to RUN_TO_POSITION mode
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        robot.setRawPower(Math.abs(speed));

        // Start movement and wait for completion or timeout
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.isAllBusy())) {

            target = robot.getAllPositions();
            telemetry.addData("Path", "Driving");
            telemetry.addData("LF", target[0]);
            telemetry.addData("RF", target[1]);
            telemetry.addData("LB", target[2]);
            telemetry.addData("RB", target[3]);
            telemetry.update();
        }

        // Stop all motion
        robot.stop();

        // Set motors back to RUN_USING_ENCODER mode
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runTestAuto() {
        telemetry.addLine("Running Test Auto Sequence...");
        telemetry.update();

        //robot.setPower(-.3, .3, -.3, .3);
       MoveForward(0.5, 12, 5);
        MoveRight(0.5, 12, 5);
        MoveBackward(0.5, 12, 5);
        MoveLeft(0.5, 12, 5);



        /*
        Turnleft(.3, 10, 2); //turns a bit to the left (haven't tested yet)
        //shoot and score
        Turnright(.3, 10, 2);
        MoveBackward(.5, 55, 5);
        MoveLeft(.5, 40, 5);
        //get artifacts from human player
        MoveRight(.5, 45, 5);
        MoveForward(.5, 60, 5);
        //shoot and score
        MoveBackward(.5, 40, 5);
        MoveLeft(.5, 15, .5); //end in park spot

        */
        sleep(5000);
    }

    public void MoveForward(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
    }

    public void MoveBackward(double speed, double inches, double timeoutS) {
        encoderDrive(speed, -inches, -inches, -inches, -inches, timeoutS);
    }

    public void MoveRight(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    public void MoveLeft(double speed, double inches, double timeoutS) {
        encoderDrive(speed, -inches, inches, inches, -inches, timeoutS);
    }

    public void Turnleft(double speed, double inches, double timeoutS) {
        encoderDrive(speed, -inches, inches, -inches, inches, timeoutS);
    }

    public void Turnright(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, -inches, inches, -inches, timeoutS);
    }

    public void telemetryChoice() {
        if (alliance_color == kBLUE) {
            telemetry.addData("Current Alliance", "Blue");
        } else if (alliance_color == kRED) {
            telemetry.addData("Current Alliance", "Red");
        } else {
            telemetry.addData("Current Alliance", "None");
        }

        if (current_start_location == kSTART_LOCATION_1) {
            telemetry.addData("Start Position", "Position 1");
        } else if (current_start_location == kSTART_LOCATION_2) {
            telemetry.addData("Start Position", "Position 2");
        } else if (current_start_location == kSTART_LOCATION_3) {
            telemetry.addData("Start Position", "Position 3");
        } else {
            telemetry.addData("Start Position", "None");
        }

        telemetry.update();
    }
}

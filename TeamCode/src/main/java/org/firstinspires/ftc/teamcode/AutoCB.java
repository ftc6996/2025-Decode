package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@Autonomous(name = "AutoShortBlue")
@Disabled
public class AutoCB extends LinearOpMode {

    /// move forward 20 inches, triple shot at 800rpm hood all way down?
    private ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive mecanumDrive;
    private Robot robot;
    private Launcher launcher;
    private static final int kNOT_SET = -1;
    private static final int kSTART_LOCATION_1 = 0;
    private static final int kSTART_LOCATION_2 = 1;
    private static final int kSTART_LOCATION_3 = 2;
    public int current_start_location = kNOT_SET;

    private static final int kBLUE = 0;
    private static final int kRED = 1;
    public int alliance_color = kNOT_SET;

    public enum AutoState {
        IDLE,
        SHOOT1,
        SHOOT2,
        SHOOT3,
        MOVE,
        MOVE_TO_SPIKE1,
        STOP;
    }

    public AutoLMB.AutoState autoState = AutoLMB.AutoState.IDLE;
    boolean isMoving = false;

    @Override
    public void runOpMode() {
        mecanumDrive = new MecanumDrive();
        mecanumDrive.init(hardwareMap);

        launcher = new Launcher();
        launcher.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Version", "1");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        runtime.reset();

        boolean finished = false;


        while (opModeIsActive() && !finished) {
            telemetry.addData("autoState", autoState);
            telemetry.addData("finished", finished);
            telemetry.addData("Lancher State", launcher.launchState);
            telemetry.addData("target v", launcher.getTargetVelocity());
            telemetry.addData("current v", launcher.getFlyWheelVelocity());
            telemetry.update();
            launcher.process();


            switch (autoState){
                case IDLE:
                    launcher.target_velocity = 880;
                    launcher.setFlyWheelVelocity(880);
                    if (runtime.seconds()> 14) {
                        autoState = AutoLMB.AutoState.SHOOT1;
                    }
                    break;
                case SHOOT1:
                    //start the shooting process
                    launcher.rapidFire = true;
                    launcher.shoot(true, 880, 1);
                    autoState =  AutoLMB.AutoState.SHOOT2;

                    break;
                case SHOOT2:
                    if (!launcher.isShotRequested())
                    {
                        autoState = AutoLMB.AutoState.STOP;
                    }
                    break;
                case SHOOT3:
                    if (launcher.launchState == Launcher.LaunchState.IDLE) {
                        launcher.shoot(true, 880, 1);
                    }
                    if (!launcher.isShotRequested())
                    {
                        autoState = AutoLMB.AutoState.MOVE;
                    }
                    break;
                case MOVE:
                    //set the move command
                    if (!isMoving)
                    {
                        isMoving = true;

                        //MoveRight(.8, 28, 5);
                        sleep(30000);
                        autoState = AutoLMB.AutoState.STOP;
                        launcher.setIntakeMotor(.8);


                    }
                    break;
                case MOVE_TO_SPIKE1:
                    if (!isMoving)
                    {
                        isMoving = true;
                        MoveForward(.3, 20, 5);
                        autoState = AutoLMB.AutoState.STOP;
                    }
                    break;
                case STOP:
                    isMoving = false;
                    finished = true;
                    break;
            }

        }

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
        int [] target = mecanumDrive.getAllPositions();
        target[0] += (int)(leftFrontInches * COUNTS_PER_INCH);
        target[1] += (int)(rightFrontInches * COUNTS_PER_INCH);
        target[2] += (int)(leftBackInches * COUNTS_PER_INCH);
        target[3] += (int)(rightBackInches * COUNTS_PER_INCH);

        // Set target positions
        mecanumDrive.setTargetPosition(target);

        // Set to RUN_TO_POSITION mode
        mecanumDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        mecanumDrive.setRawPower(Math.abs(speed));

        // Start movement and wait for completion or timeout
        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (mecanumDrive.isAllBusy())) {

            target = mecanumDrive.getAllPositions();
            telemetry.addData("Path", "Driving");
            telemetry.addData("LF", target[0]);
            telemetry.addData("RF", target[1]);
            telemetry.addData("LB", target[2]);
            telemetry.addData("RB", target[3]);
            telemetry.update();
        }

        // Stop all motion
        mecanumDrive.stop();

        // Set motors back to RUN_USING_ENCODER mode
        mecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        isMoving = false;
    }

    public void runTestAuto() {
        telemetry.addLine("Running Test Auto Sequence...");
        telemetry.update();

        //robot.setPower(-.3, .3, -.3, .3);



        launcher.shoot(true, 800, 1);

        MoveForward(.8, 9, 5);


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

    public void MoveBackward(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, inches, inches, inches, timeoutS);
    }

    public void MoveForward(double speed, double inches, double timeoutS) {
        encoderDrive(speed, -inches, -inches, -inches, -inches, timeoutS);
    }

    public void MoveLeft(double speed, double inches, double timeoutS) {
        encoderDrive(speed, inches, -inches, -inches, inches, timeoutS);
    }

    public void MoveRight(double speed, double inches, double timeoutS) {
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
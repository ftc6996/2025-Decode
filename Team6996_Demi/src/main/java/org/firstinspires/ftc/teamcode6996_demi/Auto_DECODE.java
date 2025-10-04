package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "Auto_DECODE")
public class Auto_DECODE extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumDrive robot;

    private enum Alliance {
        NOT_SET,
        BLUE,
        RED;
    }
    private Alliance alliance = Alliance.NOT_SET;

    private enum Location {
        NOT_SET,
        START_LOCATION_1,
        START_LOCATION_2,
        START_LOCATION_3;
    }
    private Location start_location = Location.NOT_SET;

    public enum AprilTags {
        ALL(-1), BLUE_GOAL(20), RED_GOAL(24),
        OBELISK_GPP(21), OBELISK_PGP(22), OBELISK_PPG(23);

        private AprilTags(final int id)
        {
            this.id = id;
        }
        public int getID()
        {
            return id;
        }
        private int id;
    }

    private static final int kNOT_SET = -1;
    private static final int kSTART_LOCATION_1 = 0;
    private static final int kSTART_LOCATION_2 = 1;
    private static final int kSTART_LOCATION_3 = 2;
    public int current_start_location = kNOT_SET;

    private static final int kBLUE = 0;
    private static final int kRED = 1;
    public int alliance_color = kNOT_SET;

    public Gamepad saved_gamepad1 = new Gamepad();
    public String audience = "none";
    public String backside = "none";
    public String position = "none";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Hardware map setup
        robot = new MecanumDrive();
        robot.init(hardwareMap);
        // Constants for wheel diameter am-4763 ?
        robot.setWheelDiameter(100);
        // Constants for encoder counts for AM-2964 AndyMark NeveRest 40 (40:1)
        robot.setMotorTicksPerRev(1120);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.x || gamepad2.x) {
            alliance_color = kBLUE;
            alliance = Alliance.BLUE;
        }
        if (gamepad1.b || gamepad2.b) {
            alliance_color = kRED;
            alliance = Alliance.RED;
        }
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            current_start_location = kSTART_LOCATION_1;//
            start_location = Location.START_LOCATION_1;
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            current_start_location = kSTART_LOCATION_2;//
            start_location = Location.START_LOCATION_2;
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            current_start_location = kSTART_LOCATION_3;//
            start_location = Location.START_LOCATION_3;
        }
        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            current_start_location = kNOT_SET;
            start_location = Location.NOT_SET;
        }
        telemetryChoice();
    }
    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.stop();
    }

    /*
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
*/
    public void telemetryChoice() {
        if (alliance_color == kBLUE) {
            telemetry.addData("Current Alliance", "Blue");
        } else if (alliance_color == kRED) {
            telemetry.addData("Current Alliance", "Red");
        } else {
            telemetry.addData("Current Alliance", "None");
        }
        telemetry.addData("Current Alliance", alliance);

        if (current_start_location == kSTART_LOCATION_1) {
            telemetry.addData("Start Position", "Position 1");
        } else if (current_start_location == kSTART_LOCATION_2) {
            telemetry.addData("Start Position", "Position 2");
        } else if (current_start_location == kSTART_LOCATION_3) {
            telemetry.addData("Start Position", "Position 3");
        } else {
            telemetry.addData("Start Position", "None");
        }
        telemetry.addData("Start Position", start_location);

        telemetry.update();
    }
}

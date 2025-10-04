package org.firstinspires.ftc.teamcode6996_demi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

@Autonomous(name = "Auto_DECODE")
public class Auto_DECODE extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2;
    final double FEED_TIME = 0.20;
    int shotsToFire = 3; //The number of shots to fire in this auto.
    double robotRotationAngle = 45;

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

    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private LaunchState launchState;

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }
    private AutonomousState autonomousState;

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
            alliance = Alliance.BLUE;
        }
        if (gamepad1.b || gamepad2.b) {
            alliance = Alliance.RED;
        }
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            start_location = Location.START_LOCATION_1;
        }
        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            start_location = Location.START_LOCATION_2;
        }
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            start_location = Location.START_LOCATION_3;
        }
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed() ) {

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
        double DRIVE_SPEED = .5;

        switch (autonomousState){
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                       // launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:

                if(robot.drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)){
                    robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                if(alliance == Alliance.RED){
                    robotRotationAngle = 45;
                } else if (alliance == Alliance.BLUE){
                    robotRotationAngle = -45;
                }

                /*
                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                    robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }

                 */
                break;

            case DRIVING_OFF_LINE:
                if(robot.drive(DRIVE_SPEED, -26, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        outputPositions("Current", robot.getAllPositions());
        outputPositions("Target", robot.getAllTargetPositions());
        telemetry.update();
    }

    public void outputPositions(String label, int [] position)
    {
        telemetry.addData(label, "LF (%d), RF (%d), LB (%d), RB (%d)",
                position[0], position[1], position[2],position[3]);
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

    boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                /*
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    //leftFeeder.setPower(1);
                    //rightFeeder.setPower(1);
                    feederTimer.reset();
                }

                 */
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                   // leftFeeder.setPower(0);
                    //rightFeeder.setPower(0);

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    public void telemetryChoice() {
        telemetry.addData("Current Alliance", alliance);
        telemetry.addData("Start Position", start_location);
        telemetry.update();
    }
}

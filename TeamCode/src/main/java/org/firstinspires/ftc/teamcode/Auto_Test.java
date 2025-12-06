package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto_Test")
public class Auto_Test extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int shotsToFire = 3; //The number of shots to fire in this auto.
    double robotRotationAngle = 45;

    private Robot robot;
    //private Launcher launcher;

    private enum Alliance {
        NOT_SET,
        BLUE,
        RED
    }
    private Alliance alliance = Alliance.NOT_SET;

    private enum Location {
        NOT_SET,
        START_LOCATION_1,
        START_LOCATION_2,
        START_LOCATION_3
    }
    private Location start_location = Location.NOT_SET;

    public enum AprilTags {
        ANY(-1), NONE(0),
        BLUE_GOAL(20), RED_GOAL(24),
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

    private enum AutonomousState {
        START,
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE
    }
    private AutonomousState autonomousState = AutonomousState.START;

    public Gamepad saved_gamepad1 = new Gamepad();

    private static final boolean USE_WEBCAM = false;  // Set true to use a webcam, or false for a phone camera
    private static final AprilTags DESIRED_TAG_ID = AprilTags.ANY;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private boolean targetFound = false;    // Set to true when an AprilTag target is detected
    private AprilTags targetFoundTag = AprilTags.NONE;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Hardware map setup
        robot = new Robot();
        robot.init(hardwareMap);
        initAprilTag();



        //launcher =  new Launcher();
        //launcher.init(hardwareMap);

        // Constants for wheel diameter am-4763 ?
        robot.mecanumDrive.setWheelDiameter(100);
        // Constants for encoder counts for AM-2964 AndyMark NeveRest 40 (40:1)
        robot.mecanumDrive.setMotorTicksPerRev(1120);

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
        autonomousState = AutonomousState.START;
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    public double clamp(double value, double min, double max){
        return Math.max(min, Math.min(max,value));
    }

    @Override
    public void loop() {
        //PinPoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
        // Setup a variable for each drive wheel to save power level for telemetry
        int Ytiles = 0;// tiles
        int Ymm = 100;// mm
        int Xtiles = 0;// tiles
        int Xmm = 0;// mm
        double stopRange = 50; // mm
        double stopTurnRange = 5;//deg
        double targetHeading = Math.toRadians(-50);// angle that the robot ends at in deg

        double K = 0.005;
        double kTurn = 1.5;//sensitivity of turn. try a range of 0.1-0.4

        //don't change anymore variables

        double turnPower = 0;

        targetHeading = Math.toRadians(targetHeading);
        double desiredYDistance = Ymm+(Ytiles*609.6);
        double desiredXDistance = Xmm+(Xtiles*609.6);
        robot.update();
        double PinPointx = robot.mecanumDrive.getPinpointPosition().getX(DistanceUnit.MM);
        double PinPointy = robot.mecanumDrive.getPinpointPosition().getY(DistanceUnit.MM);
        double currentHeadingRad = Math.toRadians(robot.mecanumDrive.getPinpointPosition().getHeading(AngleUnit.DEGREES)); // from Pinpoint
        double currentHeadingDeg = robot.mecanumDrive.getPinpointPosition().getHeading(AngleUnit.DEGREES); // from Pinpoint

        /// ////////////note
        double dx = desiredXDistance - PinPointx;
        double dy = desiredYDistance - PinPointy;
        //targetHeading = Math.atan2(dy,dx);
        double distanceError = Math.sqrt(dx*dx + dy*dy);
        double headingError = targetHeading - currentHeadingRad;
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        if (distanceError < stopRange) {
            robot.move(0, 0, 0);
            return;
        }

        double slowFactor = distanceError / 300.0;
        slowFactor = Math.min(1.0, Math.max(0.2, slowFactor));
        if (currentHeadingDeg<=stopTurnRange){
            turnPower = 0;
        }else{
            turnPower = headingError * kTurn;
            turnPower = Math.max(-0.4, Math.min(0.4, turnPower));
        }
        /// ////////////////

        double oPid = K * (Math.sqrt(Math.pow((PinPointy - desiredYDistance), 2) + (Math.pow((PinPointx - desiredXDistance), 2))));
        telemetry.addData("oPid", oPid);
        double robotTarget = Math.atan2((desiredYDistance - PinPointy),(desiredXDistance - PinPointx));

        /// ///////////////note
        double xPower = slowFactor * oPid * Math.cos(robotTarget);
        double yPower = slowFactor * oPid * Math.sin(robotTarget);

        xPower = clamp(xPower, -1, 1);
        yPower = clamp(yPower, -1, 1);
        turnPower = Math.max(-0.4, Math.min(0.4, turnPower));

        /// ///////////////

        telemetry.addData("distanceError", distanceError);
        telemetry.addData("slowFactor", slowFactor);
        robot.move(yPower, -xPower, turnPower);

        telemetry.addData("Y distance goal in MM", desiredYDistance);
        telemetry.addData("X distance goal in MM", desiredXDistance);
        telemetry.addData("currentHeading in DEG", currentHeadingDeg);
        //telemetry.addData("distance in MM", robot.PinPoint.getPosition().getX(DistanceUnit.MM));

        //this basically only happens once to find the obelisk for the game
        /*if (!targetFound)
        {
            lookingForObTag();
        }*/

        //lookingForGoalTag(alliance);

/*        switch (autonomousState){
            case LAUNCH:
                launcher.launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(launcher.launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        launcher.setVelocity(0);
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

                //if(robot.rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                //    robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                //}
                break;

            case DRIVING_OFF_LINE:
                if(robot.drive(DRIVE_SPEED, -26, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }

                break;
            default:
                break;
        }*/

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("Obelisk", targetFoundTag);// targetFound ? desiredTag.id : "NONE");
        //telemetry.addData("LauncherState", launcher.getLaunchState());
        outputPositions("Current", robot.mecanumDrive.getAllPositions());
        outputPositions("Target", robot.mecanumDrive.getAllTargetPositions());
        telemetry.addData("positionX", Math.round(robot.mecanumDrive.getPinpointPosition().getX(DistanceUnit.MM)));
        telemetry.addData("positionY", Math.round(robot.mecanumDrive.getPinpointPosition().getY(DistanceUnit.MM)));
        telemetry.addData("heading",Math.round(robot.mecanumDrive.getPinpointPosition().getHeading(AngleUnit.DEGREES)));
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
        robot.mecanumDrive.stop();
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             double timeoutS) {


        // Constants for encoder counts for AndyMark NeveRest 40 (40:1)
        final double COUNTS_PER_MOTOR_REV = 1120; // AM-2964 with 40:1 gearbox
        final double WHEEL_DIAMETER_INCHES = 4.0;  // Wheel diameter in inches
        final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

        // Calculate target positions
        int [] target = robot.mecanumDrive.getAllPositions();
        target[0] += (int)(leftFrontInches * COUNTS_PER_INCH);
        target[1] += (int)(rightFrontInches * COUNTS_PER_INCH);
        target[2] += (int)(leftBackInches * COUNTS_PER_INCH);
        target[3] += (int)(rightBackInches * COUNTS_PER_INCH);

        // Set target positions
        robot.mecanumDrive.setTargetPosition(target);

        // Set to RUN_TO_POSITION mode
        robot.mecanumDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set power
        robot.mecanumDrive.setRawPower(Math.abs(speed));

        // Start movement and wait for completion or timeout
        runtime.reset();
        /*
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
*/
        // Stop all motion
        robot.mecanumDrive.stop();

        // Set motors back to RUN_USING_ENCODER mode
        robot.mecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
///////////////////////////////////////////////////////////////////////////////////////////

    public void telemetryChoice() {
        telemetry.addData("Version", "4");
        telemetry.addData("Current Alliance", alliance);
        telemetry.addData("Start Position", start_location);
        telemetry.update();
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void mySleep(int milliseconds)
    {
        ElapsedTime timer = new ElapsedTime();
        int x = 0;
        while(timer.milliseconds() < milliseconds)
        {
            x++;
        }
    }
    /*
         Manually set the camera gain and exposure.
         This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        runtime.startTime();
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                mySleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            mySleep(50);
        }
        exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        mySleep(20);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        mySleep(20);
    }

    private void lookingForObTag() {
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == AprilTags.OBELISK_GPP.getID()) ||
                    (detection.id == AprilTags.OBELISK_PGP.getID()) ||
                    (detection.id == AprilTags.OBELISK_PPG.getID()))
                {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    if (detection.id == AprilTags.OBELISK_GPP.getID())
                    {
                        targetFoundTag = AprilTags.OBELISK_GPP;
                    }
                    if (detection.id == AprilTags.OBELISK_PGP.getID())
                    {
                        targetFoundTag = AprilTags.OBELISK_PGP;
                    }
                    if (detection.id == AprilTags.OBELISK_PPG.getID())
                    {
                        targetFoundTag = AprilTags.OBELISK_PPG;
                    }
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
    private void lookingForGoalTag(Alliance alliancePicked) {

        boolean driveToRequested = false;
        double heading = robot.mecanumDrive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double heading_deg = robot.mecanumDrive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
        final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
        double drive  = 0;
        double strafe = 0;
        double twist  = 0;
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        targetFound = false;
        desiredTag = null;
        int lookingForID = -1;
        if (alliancePicked == Alliance.BLUE)
        {
            lookingForID = AprilTags.BLUE_GOAL.getID();
        }
        else if (alliancePicked == Alliance.RED)
        {
            lookingForID = AprilTags.RED_GOAL.getID();
        }
        else
        {
            return;
        }
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if (detection.id == lookingForID)
                {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    if (driveToRequested && targetFound) {

                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double  headingError    = desiredTag.ftcPose.bearing;
                        double  yawError        = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        twist   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, twist);
                    }
                    robot.move(drive, strafe, twist);

                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }
}
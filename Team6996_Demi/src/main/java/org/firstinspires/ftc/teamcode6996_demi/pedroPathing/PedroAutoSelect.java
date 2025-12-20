package org.firstinspires.ftc.teamcode6996_demi.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.changes;
import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.draw;
import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.follower;
import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.stopRobot;
import static org.firstinspires.ftc.teamcode6996_demi.pedroPathing.PedroAutoSelect.telemetryM;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.math.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the Tuning class. It contains a selection menu for various tuning OpModes.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 6/26/2025
 */
@Configurable
@TeleOp(name = "PedroAutoSelect", group = "Pedro Pathing")
public class PedroAutoSelect extends SelectableOpMode {
    public static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    public PedroAutoSelect() {
        super("Select a Tuning OpMode", a -> {
            a.folder("Blue", b -> {
                b.folder("Small Launch Zone",c ->{
                    c.add("Shoot 3, prepare to classify", S3PtC::new);
                    //c.add("", ForwardTuner::new);
                });
                b.folder("Large Launch Zone", c ->{
                    //c.add("Localization Test", LocalizationTest::new);
                    //c.add("Forward Tuner", ForwardTuner::new);
                });
            });
            a.folder("Red", b -> {
                b.folder("Small Launch Zone",c ->{
                    //c.add("Localization Test", LocalizationTest::new);
                    //c.add("Forward Tuner", ForwardTuner::new);
                });
                b.folder("Large Launch Zone", c ->{
                    //c.add("Localization Test", LocalizationTest::new);
                    //c.add("Forward Tuner", ForwardTuner::new);
                });
            });
            a.folder("Test", b -> {
                //p.add("Translational Tuner", TranslationalTuner::new);
                //p.add("Heading Tuner", HeadingTuner::new);
                //p.add("Drive Tuner", DriveTuner::new);
                //p.add("Line Tuner", Line::new);
                //p.add("Centripetal Tuner", CentripetalTuner::new);
            });
            /*
            s.folder("Tests", p -> {
                p.add("Line", Line::new);
                p.add("Triangle", Triangle::new);
                p.add("Circle", Circle::new);
            });*/
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Drawing.init();
    }

    @Override
    public void onLog(List<String> lines) {}

    public static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        Drawing.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    public static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}

class S3PtC extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        MOVE_TWO,
        MOVE_THREE,
        MOVE_FOUR,
        MOVE_FIVE,
        MOVE_SIX,
        MOVE_SEVEN,
        MOVE_EIGHT,
        MOVE_NINE,
        MOVE_TEN,
        END
    }
    PathState pathState;

    public enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(56,8,Math.toRadians(90));
    private final Pose shootPose = new Pose(56,21,Math.toRadians(110));
    private final Pose threePose = new Pose(41,35.5,Math.toRadians(180));
    private final Pose fourPose = new Pose(24.5,35.5,Math.toRadians(180));
    private final Pose fivePose = new Pose(56,21,Math.toRadians(110));
    private final Pose sixPose = new Pose(41,60,Math.toRadians(180));
    private final Pose sevenPose = new Pose(24.5,60,Math.toRadians(180));
    private final Pose eightPose = new Pose(56,21,Math.toRadians(110));
    private final Pose ninePose = new Pose(41,84,Math.toRadians(180));
    private final Pose tenPose = new Pose(24.5,84,Math.toRadians(180));
    private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));

    private PathChain driveStartPosShootPos;
    private PathChain moveTwo;
    private PathChain moveThree;
    private PathChain moveFour;
    private PathChain moveFive;
    private PathChain moveSix;
    private PathChain moveSeven;
    private PathChain moveEight;
    private PathChain moveNine;
    private PathChain moveTen;
    private PathChain moveEnd;


    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, threePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), threePose.getHeading())
                .build();
        moveThree = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, threePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), threePose.getHeading())
                .build();
        moveFour = follower.pathBuilder()
                .addPath(new BezierLine(threePose, fourPose))
                .setLinearHeadingInterpolation(threePose.getHeading(), fourPose.getHeading())
                .build();
        moveFive = follower.pathBuilder()
                .addPath(new BezierLine(fourPose, fivePose))
                .setLinearHeadingInterpolation(fourPose.getHeading(), fivePose.getHeading())
                .build();
        moveSix = follower.pathBuilder()
                .addPath(new BezierLine(fivePose, sixPose))
                .setLinearHeadingInterpolation(fivePose.getHeading(), sixPose.getHeading())
                .build();
        moveSeven = follower.pathBuilder()
                .addPath(new BezierLine(sixPose, sevenPose))
                .setLinearHeadingInterpolation(sixPose.getHeading(), sevenPose.getHeading())
                .build();
        moveEight = follower.pathBuilder()
                .addPath(new BezierLine(sevenPose, eightPose))
                .setLinearHeadingInterpolation(sevenPose.getHeading(), eightPose.getHeading())
                .build();
        moveNine = follower.pathBuilder()
                .addPath(new BezierLine(eightPose, ninePose))
                .setLinearHeadingInterpolation(eightPose.getHeading(), ninePose.getHeading())
                .build();
        moveTen = follower.pathBuilder()
                .addPath(new BezierLine(ninePose, tenPose))
                .setLinearHeadingInterpolation(ninePose.getHeading(), tenPose.getHeading())
                .build();
        moveEnd = follower.pathBuilder()
                .addPath(new BezierLine(tenPose, endPose))
                .setLinearHeadingInterpolation(tenPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
                    pathState = PathState.MOVE_TWO;
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_TWO;
                    }
                }
                break;
            case MOVE_TWO:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 2");
                    follower.followPath(moveThree,true);
                    pathState = PathState.MOVE_THREE;
                }
                break;
            case MOVE_THREE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 3");
                    follower.followPath(moveFour,true);
                    pathState = PathState.MOVE_FOUR;
                }
                break;
            case MOVE_FOUR:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 4");
                    follower.followPath(moveFive,true);
                    pathState = PathState.MOVE_FIVE;
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_FIVE;
                    }
                }
                break;
            case MOVE_FIVE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 5");
                    follower.followPath(moveSix,true);
                    pathState = PathState.MOVE_SIX;
                }
                break;
            case MOVE_SIX:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 6");
                    follower.followPath(moveSeven,true);
                    pathState = PathState.MOVE_SEVEN;
                }
                break;
            case MOVE_SEVEN:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 7");
                    follower.followPath(moveEight,true);
                    pathState = PathState.MOVE_EIGHT;
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_EIGHT;
                    }
                }
                break;
            case MOVE_EIGHT:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 8");
                    follower.followPath(moveNine,true);
                    pathState = PathState.MOVE_NINE;
                }
                break;
            case MOVE_NINE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 9");
                    follower.followPath(moveTen,true);
                    pathState = PathState.MOVE_TEN;
                }
                break;
            case MOVE_TEN:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 10");
                    follower.followPath(moveEnd,true);
                    pathState = PathState.END;
                }
                break;
            case END:
                if(!follower.isBusy()){
                    telemetry.addLine("All Paths Done");
                }
                break;
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    public void stateLaunchUpdate(){
        switch (launchState){
            case STOPED:
                //set launch motor to 0 velocity
                //set turret_hood_servo to lowered position
                break;
            case IDLE:
                //set launch motor to Idle velocity
                //set turret_hood_servo to lowered position
                break;
            case CLOSE_SPEED:
                //set launch motor to large launch zone set velocity
                //set turret_hood_servo to large launch zone position
                break;
            case FAR_SPEED:
                //set launch motor to small launch zone velocity
                //set turret_hood_servo to small launch zone position
                break;
            case LAUNCH:
                //activate turret_feeder_servo or pass through or both to get artifact to wheel
                launchState = LaunchState.IDLE;
                break;
            default:
                telemetry.addLine("No Launch State Commanded");
                break;
        }
    }

    public void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    public void setLaunchState(LaunchState newLaunchState){
        launchState = newLaunchState;
    }

    @Override
    public void init(){
        pathTimer = new Timer();
        opModeTimer = new Timer();
        mecanumDrive = new MecanumDrive();

        mecanumDrive.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        Constants.initOthers(hardwareMap);

        launchState = LaunchState.STOPED;

        buildPaths();
        follower.setPose(startPose);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(PathState.DRIVE_STARTPOS_SHOOT_POS);
        setLaunchState(launchState);
    }
    @Override
    public void loop(){
        follower.update();
        statePathUpdate();
        stateLaunchUpdate();
        int [] target = mecanumDrive.getAllPositions();
        telemetry.addData("LF", target[0]);
        telemetry.addData("RF", target[1]);
        telemetry.addData("LB", target[2]);
        telemetry.addData("RB", target[3]);
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}
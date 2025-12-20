package org.firstinspires.ftc.teamcode6996_demi.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode6996_demi.pedroPathing.LargeAuto.BSS9PtC_Single;

import java.util.ArrayList;
import java.util.List;

@Configurable
@Autonomous(name = "PedroAutoSelect", group = "Pedro Pathing")
public class PedroAutoSelect extends SelectableOpMode {
    private static Follower follower;

    @IgnoreConfigurable
    static PoseHistory poseHistory;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    @IgnoreConfigurable
    static ArrayList<String> changes = new ArrayList<>();

    private PedroAutoSelect() {
        super("Select a Tuning OpMode", a -> {
            a.folder("Blue", b -> {
                b.folder("Small Launch Zone",c ->{
                    c.add("Shoot 9, prepare to classify", BSS9PtC_Single::new);
                    c.add("Shoot 3, leave", BSS3L::new);
                });
                b.folder("Large Launch Zone", c ->{
                    c.add("Shoot 9, prepare to classify", BLS9PtC::new);
                    c.add("Shoot 3, leave", BLS3L::new);
                });
            });
            a.folder("Red", b -> {
                b.folder("Small Launch Zone",c ->{
                    c.add("Shoot 9, prepare to classify", RSS9PtC::new);
                    c.add("Shoot 3, leave", RSS3L::new);
                });
                b.folder("Large Launch Zone", c ->{
                    c.add("Shoot 9, prepare to classify", RLS9PtC::new);
                    c.add("Shoot 3, leave", RLS3L::new);
                });
            });
            a.folder("Test", b -> {

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

    private static void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    private static void draw() {
        Drawing.drawDebug(follower);
    }

    /** This creates a full stop of the robot by setting the drive motors to run at 0 power. */
    private static void stopRobot() {
        follower.startTeleopDrive(true);
        follower.setTeleOpDrive(0,0,0,true);
    }
}

class BSS9PtC extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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

    private enum LaunchState{
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


    private void buildPaths(){
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

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
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

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class BSS3L extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        MOVE_TWO,
        MOVE_THREE,
        //MOVE_FOUR,
        //MOVE_FIVE,
        //MOVE_SIX,
        //MOVE_SEVEN,
        //MOVE_EIGHT,
        //MOVE_NINE,
        //MOVE_TEN,
        //END
    }
    PathState pathState;

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(18,121.5,Math.toRadians(-37));
    private final Pose shootPose = new Pose(67,84.25,Math.toRadians(135));
    private final Pose threePose = new Pose(53.25,70,Math.toRadians(135));
    //private final Pose fourPose = new Pose(24.5,35.5,Math.toRadians(180));
    //private final Pose fivePose = new Pose(56,21,Math.toRadians(110));
    //private final Pose sixPose = new Pose(41,60,Math.toRadians(180));
    //private final Pose sevenPose = new Pose(24.5,60,Math.toRadians(180));
    //private final Pose eightPose = new Pose(56,21,Math.toRadians(110));
    //private final Pose ninePose = new Pose(41,84,Math.toRadians(180));
    //private final Pose tenPose = new Pose(24.5,84,Math.toRadians(180));
    //private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));

    private PathChain driveStartPosShootPos;
    private PathChain moveTwo;
    private PathChain moveThree;
    //private PathChain moveFour;
    //private PathChain moveFive;
    //private PathChain moveSix;
    //private PathChain moveSeven;
    //private PathChain moveEight;
    //private PathChain moveNine;
    //private PathChain moveTen;
    //private PathChain moveEnd;


    private void buildPaths(){
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
        /*moveFour = follower.pathBuilder()
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
                .build();*/
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
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
                    if(!follower.isBusy()){
                        telemetry.addLine("All Paths Done");
                    }
                }
                break;
            /*case MOVE_FOUR:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 4");
                    follower.followPath(moveFive,true);
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
                break;*/
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class BLS9PtC extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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
        //END
    }
    PathState pathState;

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(18,121.5,Math.toRadians(143));
    private final Pose shootPose = new Pose(67,83.5,Math.toRadians(180));
    private final Pose threePose = new Pose(24.5,83.5,Math.toRadians(180));
    private final Pose fourPose = new Pose(66.5,83.5,Math.toRadians(135));
    private final Pose fivePose = new Pose(38.25,60,Math.toRadians(180));
    private final Pose sixPose = new Pose(23,60,Math.toRadians(180));
    private final Pose sevenPose = new Pose(66.5,84,Math.toRadians(135));
    private final Pose eightPose = new Pose(34.25,35.5,Math.toRadians(180));
    private final Pose ninePose = new Pose(23.5,35.5,Math.toRadians(180));
    private final Pose tenPose = new Pose(18.5,71.5,Math.toRadians(90));
    //private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));

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
    //private PathChain moveEnd;


    private void buildPaths(){
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
        /*moveEnd = follower.pathBuilder()
                .addPath(new BezierLine(tenPose, endPose))
                .setLinearHeadingInterpolation(tenPose.getHeading(), endPose.getHeading())
                .build();*/
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
               /* if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                    launchState = LaunchState.FAR_SPEED;
                }
                if(launchState == LaunchState.IDLE){
                    pathState = PathState.SHOOT_PRELOAD;
                }*/
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
                    pathState = PathState.MOVE_TWO;
                }
                break;
            case MOVE_TWO:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 2");
                    follower.followPath(moveThree,true);
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_THREE;
                    }
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
                }
                break;
            case MOVE_FIVE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 5");
                    follower.followPath(moveSix,true);
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_SIX;
                    }
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
                    if(!follower.isBusy()){
                        telemetry.addLine("All Paths Done");
                    }
                }
                break;
            /*case END:
                if(!follower.isBusy()){
                    telemetry.addLine("All Paths Done");
                }
                break;*/
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class BLS3L extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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

    private enum LaunchState{
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


    private void buildPaths(){
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

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
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

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
///       //////          ////      ////////////////////////////////////////////////////////////////////
/// /////  ///// ///////////// ////   //////////////////////////////////////////////////////////////////
/// //////  //// ///////////// //////  /////////////////////////////////////////////////////////////////
/// //////  //// ///////////// /////// /////////////////////////////////////////////////////////////////
/// /////  /////          //// /////// /////////////////////////////////////////////////////////////////
///      /////// ///////////// /////// /////////////////////////////////////////////////////////////////
/// ///  /////// ///////////// //////  /////////////////////////////////////////////////////////////////
/// ////  ////// ///////////// ////   /////////////////////////////////////////////////////////////////
/// /////  /////          ////     /////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
class RSS9PtC extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(88,8,Math.toRadians(90));
    private final Pose shootPose = new Pose(88,21,Math.toRadians(70));
    private final Pose threePose = new Pose(103,35.5,Math.toRadians(0));
    private final Pose fourPose = new Pose(119.5,35.5,Math.toRadians(0));
    private final Pose fivePose = new Pose(88,21,Math.toRadians(70));
    private final Pose sixPose = new Pose(103,60,Math.toRadians(0));
    private final Pose sevenPose = new Pose(119.5,60,Math.toRadians(0));
    private final Pose eightPose = new Pose(88,21,Math.toRadians(70));
    private final Pose ninePose = new Pose(103,84,Math.toRadians(0));
    private final Pose tenPose = new Pose(119.5,84,Math.toRadians(0));
    private final Pose endPose = new Pose(124.5,70.5,Math.toRadians(90));

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


    private void buildPaths(){
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

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
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

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class RSS3L extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(88,8,Math.toRadians(90));
    private final Pose shootPose = new Pose(88,27.5,Math.toRadians(110));
    /*private final Pose threePose = new Pose(41,35.5,Math.toRadians(180));
    private final Pose fourPose = new Pose(24.5,35.5,Math.toRadians(180));
    private final Pose fivePose = new Pose(56,21,Math.toRadians(110));
    private final Pose sixPose = new Pose(41,60,Math.toRadians(180));
    private final Pose sevenPose = new Pose(24.5,60,Math.toRadians(180));
    private final Pose eightPose = new Pose(56,21,Math.toRadians(110));
    private final Pose ninePose = new Pose(41,84,Math.toRadians(180));
    private final Pose tenPose = new Pose(24.5,84,Math.toRadians(180));
    private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));*/

    private PathChain driveStartPosShootPos;
    /*private PathChain moveTwo;
    private PathChain moveThree;
    private PathChain moveFour;
    private PathChain moveFive;
    private PathChain moveSix;
    private PathChain moveSeven;
    private PathChain moveEight;
    private PathChain moveNine;
    private PathChain moveTen;
    private PathChain moveEnd;*/


    private void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        /*moveTwo = follower.pathBuilder()
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
                .build();*/
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                    launchState = LaunchState.FAR_SPEED;
                }
                if(launchState == LaunchState.IDLE){
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    if(!follower.isBusy()){
                        telemetry.addLine("All Paths Done");
                    }
                }
                break;
            /*case MOVE_TWO:
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
                break;*/
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class RLS9PtC extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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
        //END
    }
    PathState pathState;

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(121,127,Math.toRadians(225));
    private final Pose shootPose = new Pose(77.5,84.25,Math.toRadians(360));
    private final Pose threePose = new Pose(119.5,83.5,Math.toRadians(360));
    private final Pose fourPose = new Pose(77.5,84.25,Math.toRadians(45));
    private final Pose fivePose = new Pose(105.75,60,Math.toRadians(360));
    private final Pose sixPose = new Pose(121,60,Math.toRadians(360));
    private final Pose sevenPose = new Pose(77.5,84.25,Math.toRadians(45));
    private final Pose eightPose = new Pose(109.75,35.5,Math.toRadians(360));
    private final Pose ninePose = new Pose(120.5,36,Math.toRadians(360));
    private final Pose tenPose = new Pose(125.5,71.5,Math.toRadians(90));
    //private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));

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
    //private PathChain moveEnd;


    private void buildPaths(){
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
        /*moveEnd = follower.pathBuilder()
                .addPath(new BezierLine(tenPose, endPose))
                .setLinearHeadingInterpolation(tenPose.getHeading(), endPose.getHeading())
                .build();*/
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
               /* if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                    launchState = LaunchState.FAR_SPEED;
                }
                if(launchState == LaunchState.IDLE){
                    pathState = PathState.SHOOT_PRELOAD;
                }*/
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
                    pathState = PathState.MOVE_TWO;
                }
                break;
            case MOVE_TWO:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 2");
                    follower.followPath(moveThree,true);
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_THREE;
                    }
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
                }
                break;
            case MOVE_FIVE:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 5");
                    follower.followPath(moveSix,true);
                    if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                        launchState = LaunchState.FAR_SPEED;
                    }
                    if(launchState == LaunchState.IDLE){
                        pathState = PathState.MOVE_SIX;
                    }
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
                    if(!follower.isBusy()){
                        telemetry.addLine("All Paths Done");
                    }
                }
                break;
            /*case END:
                if(!follower.isBusy()){
                    telemetry.addLine("All Paths Done");
                }
                break;*/
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}

class RLS3L extends OpMode {
    private MecanumDrive mecanumDrive;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private enum PathState{
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

    private enum LaunchState{
        STOPED,
        IDLE,
        FAR_SPEED,
        CLOSE_SPEED,
        LAUNCH
    }
    LaunchState launchState;

    private final Pose startPose = new Pose(126,121.5,Math.toRadians(220));
    private final Pose shootPose = new Pose(77,84.25,Math.toRadians(45));
    private final Pose threePose = new Pose(90.75,70,Math.toRadians(45));
    /*private final Pose fourPose = new Pose(24.5,35.5,Math.toRadians(180));
    private final Pose fivePose = new Pose(56,21,Math.toRadians(110));
    private final Pose sixPose = new Pose(41,60,Math.toRadians(180));
    private final Pose sevenPose = new Pose(24.5,60,Math.toRadians(180));
    private final Pose eightPose = new Pose(56,21,Math.toRadians(110));
    private final Pose ninePose = new Pose(41,84,Math.toRadians(180));
    private final Pose tenPose = new Pose(24.5,84,Math.toRadians(180));
    private final Pose endPose = new Pose(24.5,70,Math.toRadians(270));*/

    private PathChain driveStartPosShootPos;
    private PathChain moveTwo;
    /*private PathChain moveThree;
    private PathChain moveFour;
    private PathChain moveFive;
    private PathChain moveSix;
    private PathChain moveSeven;
    private PathChain moveEight;
    private PathChain moveNine;
    private PathChain moveTen;
    private PathChain moveEnd;*/


    private void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, threePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), threePose.getHeading())
                .build();
        /*moveThree = follower.pathBuilder()
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
                .build();*/
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                if (launchState != LaunchState.FAR_SPEED && launchState != LaunchState.IDLE){
                    launchState = LaunchState.FAR_SPEED;
                }
                if(launchState == LaunchState.IDLE){
                    pathState = PathState.MOVE_TWO;
                }
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
                    follower.followPath(moveTwo,true);
                    if(!follower.isBusy()){
                        telemetry.addLine("All Paths Done");
                    }
                }
                break;
            /*case MOVE_TWO:
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
                break;*/
            default:
                telemetry.addLine("No Path State Commanded");
                break;
        }
    }

    private void stateLaunchUpdate(){
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

    private void setPathState(PathState newPathState){
        pathState = newPathState;
        pathTimer.resetTimer();
    }

    private void setLaunchState(LaunchState newLaunchState){
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
    @Override
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}
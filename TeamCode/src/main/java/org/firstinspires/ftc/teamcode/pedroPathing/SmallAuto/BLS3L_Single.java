package org.firstinspires.ftc.teamcode.pedroPathing.SmallAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "BLS3L_Single", group = "Pedro Pathing Small Auto")
public class BLS3L_Single extends OpMode {
    private Robot robot;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private ElapsedTime shootTimer;
    private int numShots = 3;

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

    private final Pose startPose = new Pose(18,121.5,Math.toRadians(-37));
    private final Pose shootPose = new Pose(67,84.25,Math.toRadians(135));
    private final Pose threePose = new Pose(53.25,70,Math.toRadians(135));

    private PathChain driveStartPosShootPos;
    private PathChain moveTwo;

    private void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        moveTwo = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, threePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), threePose.getHeading())
                .build();

    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                launchState = LaunchState.CLOSE_SPEED;
                //if(!follower.isBusy())
                if (follower.atPose(shootPose, 10, 10, 10))
                {
                    pathState = PathState.SHOOT_PRELOAD;
                    shootTimer.reset();
                }
                telemetry.addData("launchState", launchState);
                telemetry.addData("pathState", pathState);
                break;
            case SHOOT_PRELOAD:
                //shooting
                if (numShots > 0)
                {
                    launchState = LaunchState.LAUNCH;
                }
                //when done shooting, set to move two
                if (launchState == LaunchState.IDLE )
                {
                    if (follower.atPose(threePose, 10, 10, 10)){
                        follower.followPath(moveTwo,true);
                    }
                }
                break;
            case MOVE_TWO:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 2");
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
                if (numShots > 0)
                {
                    if (shootTimer.seconds() > 1)
                    {
                        numShots--;
                        shootTimer.reset();
                    }
                }
                else
                {
                    launchState = LaunchState.IDLE;
                }
                //activate turret_feeder_servo or pass through or both to get artifact to wheel

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
        robot = new Robot();

        robot.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);

        //Constants.initOthers(hardwareMap);

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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("path time",pathTimer.getElapsedTimeSeconds());
    }
}
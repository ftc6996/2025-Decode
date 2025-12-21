package org.firstinspires.ftc.teamcode6996_demi.pedroPathing.SmallAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode6996_demi.pedroPathing.Constants;

@Autonomous(name = "RSS3L_Single", group = "Pedro Pathing Small Auto")
public class RSS3L_Single extends OpMode {
    private MecanumDrive mecanumDrive;
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

    private final Pose startPose = new Pose(88,8,Math.toRadians(90));
    private final Pose shootPose = new Pose(88,27.5,Math.toRadians(70));

    private PathChain driveStartPosShootPos;

    private void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    private void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                //shooting
                if (numShots > 0)
                {
                    launchState = LaunchState.LAUNCH;
                }
                //when done shooting, set to move two
                if (launchState == LaunchState.IDLE )
                {
                    if (follower.atPose(shootPose, 10, 10, 10)){
                        follower.followPath(driveStartPosShootPos,true);
                        pathState = pathState.SHOOT_PRELOAD;
                    }
                }
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()){
                    telemetry.addLine("Done Path 1");
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
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.Game.kTAG_GOAL_BLUE;
import static org.firstinspires.ftc.teamcode.Constants.Game.kTAG_GOAL_RED;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MAX_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kHOOD_MIN_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kLAUNCHER_TARGET_VELOCITY_CLOSE;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kLAUNCHER_TARGET_VELOCITY_FAR;
import static org.firstinspires.ftc.teamcode.Constants.kALLIANCE_BLUE;
import static org.firstinspires.ftc.teamcode.Constants.kALLIANCE_RED;
import static org.firstinspires.ftc.teamcode.Constants.kNOT_SET;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.util.Timer;

@Autonomous(name = "Pedro_DECODE")
public class Pedro_DECODE extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  int shotsToFire = 3; //The number of shots to fire in this auto.
  double robotRotationAngle = 45;

  private Robot robot;
  private Follower follower;
  private Timer pathTimer, opModeTimer;

  private int alliance = kNOT_SET;

  private enum Location {
    NOT_SET,
    BASIC_SMALL_ZONE,
    BASIC_BIG_ZONE
  }
  private Location start_location = Location.NOT_SET;

  private enum AutonomousState {
    IDLE,
    START,
    MOVE_TO_LAUNCH,
    LAUNCH,
    WAIT_FOR_LAUNCH,
    MOVE_OUT_OF_ZONE,
    COMPLETE,
    //BONUS MOVES
    MOVE_TO_SPIKE,
    INTAKE_AT_SPIKE,
    MOVE_TO_ZONE,
    LAUNCH_AGAIN,
    MOVE_OUT_OF_ZONE_AGAIN,
    STOP
  }
  private AutonomousState autonomousState = AutonomousState.IDLE;
  private enum PathState{
    SMALL_BLUE_START,
    SMALL_BLUE_LAUNCH,
    SMALL_RED_START,
    SMALL_RED_LAUNCH,
    LARGE_BLUE_START,
    LARGE_RED_START,
    END
  }
  private PathState pathState = null;
  private boolean isMoving = false;
  private Pose2D currentPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
  private Pose2D targetPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
  private int bonusVelocity = 0;
  private boolean bonusAuto = false;

  private double tolorance = 1;
  private final Pose startSmallBluePose = new Pose(56,8,Math.toRadians(180));
  private final Pose startSmallRedPose = new Pose(88,8,Math.toRadians(0));
  private final Pose startLargeBluePose = new Pose(56,135.5,Math.toRadians(0));
  private final Pose startLargeRedPose = new Pose(88,135.5,Math.toRadians(180));
  private final Pose launchSmallBluePose = new Pose(52,22.9,Math.toRadians(180));
  private final Pose launchSmallRedPose = new Pose(88,22.9,Math.toRadians(0));
  private final Pose endSmallBluePose = new Pose(52,36,Math.toRadians(180));
  private final Pose endSmallRedPose = new Pose(88,36,Math.toRadians(0));
  private final Pose endLargeBluePose = new Pose(56,68,Math.toRadians(180));
  private final Pose endLargeRedPose = new Pose(88,68,Math.toRadians(0));


  private PathChain SmallBluestartPoseTolaunchPose;
  private PathChain SmallRedstartPoseTolaunchPose;
  private PathChain SmallBluelaunchPoseToendPose;
  private PathChain SmallRedlaunchPoseToendPose;
  private PathChain LargeBluestartPoseToendPose;
  private PathChain LargeRedstartPoseToendPose;

  private void buildPaths(){
    SmallBluestartPoseTolaunchPose = follower.pathBuilder()
            .addPath(new BezierLine(startSmallBluePose, launchSmallBluePose))
            .setLinearHeadingInterpolation(startSmallBluePose.getHeading(), launchSmallBluePose.getHeading())
            .build();
    SmallBluelaunchPoseToendPose = follower.pathBuilder()
            .addPath(new BezierLine(launchSmallBluePose, endSmallBluePose))
            .setLinearHeadingInterpolation(launchSmallBluePose.getHeading(), endSmallBluePose.getHeading())
            .build();
    SmallRedstartPoseTolaunchPose = follower.pathBuilder()
            .addPath(new BezierLine(startSmallRedPose, launchSmallRedPose))
            .setLinearHeadingInterpolation(startSmallRedPose.getHeading(), launchSmallRedPose.getHeading())
            .build();
    SmallRedlaunchPoseToendPose = follower.pathBuilder()
            .addPath(new BezierLine(launchSmallRedPose, endSmallRedPose))
            .setLinearHeadingInterpolation(launchSmallRedPose.getHeading(), endSmallRedPose.getHeading())
            .build();
    LargeBluestartPoseToendPose = follower.pathBuilder()
            .addPath(new BezierLine(startLargeBluePose, endLargeBluePose))
            .setLinearHeadingInterpolation(startLargeBluePose.getHeading(), endLargeBluePose.getHeading())
            .build();
    LargeRedstartPoseToendPose = follower.pathBuilder()
            .addPath(new BezierLine(startLargeRedPose, endLargeRedPose))
            .setLinearHeadingInterpolation(startLargeRedPose.getHeading(), endLargeRedPose.getHeading())
            .build();
  }

  private void statePathUpdate(){
    switch(pathState){
      case SMALL_BLUE_START:
        follower.followPath(SmallBluestartPoseTolaunchPose,true);
        break;
      case SMALL_BLUE_LAUNCH:
        follower.followPath(SmallBluelaunchPoseToendPose,true);
        break;
      case SMALL_RED_START:
        follower.followPath(SmallRedstartPoseTolaunchPose,true);
        break;
      case SMALL_RED_LAUNCH:
        follower.followPath(SmallRedlaunchPoseToendPose,true);
        break;
      case LARGE_BLUE_START:
        follower.followPath(LargeBluestartPoseToendPose);
        break;
      case LARGE_RED_START:
        follower.followPath(LargeRedstartPoseToendPose);
        break;
      case END:
        telemetry.addLine("all paths done");
        break;
      default:
        telemetry.addLine("No Path State Commanded");
        break;
    }
  }

  private void setPathState(PathState newPathState){
    pathState = newPathState;
    pathTimer.resetTimer();
  }

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    // Hardware map setup
    pathTimer = new Timer();
    robot = new Robot();
    robot.init(hardwareMap);

    telemetry.addData("Status", "Initialized");
    telemetry.update();
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
   */
  @Override
  public void init_loop() {
    if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
      alliance = kALLIANCE_BLUE;
      robot.setAlliance(alliance);
      robot.launcher.target_tag = kTAG_GOAL_BLUE;
    }
    if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
      alliance = kALLIANCE_RED;
      robot.setAlliance(alliance);
      robot.launcher.target_tag = kTAG_GOAL_RED;
    }
    if (gamepad1.aWasPressed() || gamepad2.aWasPressed())
    {
        bonusAuto = !bonusAuto;
    }
    if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
      start_location = Location.BASIC_SMALL_ZONE;
    }
    if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
      start_location = Location.BASIC_BIG_ZONE;
    }
    if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
        bonusVelocity += 20;
    }
    if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
        bonusVelocity -= 20;
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
  @Override
  public void loop() {
    follower.update();
    statePathUpdate();
    //robot.update();
    //robot.process(runtime);

    if (start_location == Location.BASIC_SMALL_ZONE) {
      runBasicSmallZone();
    } else if (start_location == Location.BASIC_BIG_ZONE) {
      runBasicBigZone();
    }

    /*telemetry.addData("AutoState", autonomousState);
    //telemetry.addData("Obelisk", targetFoundTag);// targetFound ? desiredTag.id : "NONE");
    telemetry.addData("Target Vel", robot.launcher.target_velocity);
    telemetry.addData("Current Vel", robot.launcher.getFlyWheelVelocity());
    telemetry.addData("LauncherState", robot.launcher.launchState);
    //telemetry.addData("Fufiled shots", robot.launcher.numShotsFufiled);
    //telemetry.addData("Requested shots", robot.launcher.numShotsRequested);
    telemetry.addData("Turret Position:", robot.launcher.getPositon());
    //outputPositions("Current", robot.DriveTrain().getAllPositions());
    //outputPositions("Target", robot.DriveTrain().getAllPositions());
    telemetry.addData("current", Format2D(currentPos));
    telemetry.addData("target", Format2D(targetPos));
    telemetry.addData("positionX", Math.round(robot.DriveTrain().getPinpointPosition().getX(DistanceUnit.MM)));
    telemetry.addData("positionY", Math.round(robot.DriveTrain().getPinpointPosition().getY(DistanceUnit.MM)));
    telemetry.addData("heading", Math.round(robot.DriveTrain().getPinpointPosition().getHeading(AngleUnit.DEGREES)));*/
    telemetry.update();
  }

  public String Format2D(Pose2D p)
  {
    String output = String.format("x: %.2f, y: %.2f", p.getX(DistanceUnit.INCH), p.getY(DistanceUnit.INCH));

    return output;
  }
  //Small/Far Zone
  public void runBasicSmallZone() {
    //shoot
    switch (autonomousState) {
    case START:
      //this is to start the spinup
      robot.launcher.target_velocity = kLAUNCHER_TARGET_VELOCITY_FAR + bonusVelocity;
      robot.launcher.setFlyWheelVelocity(robot.launcher.target_velocity);
      robot.launcher.setHoodPosition(kHOOD_MAX_POS);

      if (runtime.seconds() > 2) {
        if (alliance == kALLIANCE_BLUE) {
          //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) - 12.5, AngleUnit.DEGREES, 0);
        }
        else {
          //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) + 15, AngleUnit.DEGREES, 0);
        }
        autonomousState = AutonomousState.MOVE_TO_LAUNCH;
      }
      break;
      case MOVE_TO_LAUNCH:
        if (!isMoving) {
          if ( alliance == kALLIANCE_BLUE) {
            setPathState(PathState.SMALL_BLUE_START);
            //robot.move(0, -.5, 0);
          }
          else {
            setPathState(PathState.SMALL_RED_START);
            //robot.move(0, .5, 0);
          }
          //start moving to X/Y
          //isMoving = true;
        } else {
          //currentPos = robot.DriveTrain().getPinpointPosition();
          //double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
          //double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

          if (follower.atPose(launchSmallBluePose, tolorance, tolorance, tolorance)||follower.atPose(launchSmallRedPose, tolorance, tolorance, tolorance))
          {
          //  robot.DriveTrain().stop();
            autonomousState = AutonomousState.LAUNCH;
          //  isMoving = false;
          }
          //we are moving, check to see if we get to destination
        }
        break;
    case LAUNCH:
      //start the shooting process
      robot.launcher.rapidFire = true;
      robot.shoot(true, kLAUNCHER_TARGET_VELOCITY_FAR + bonusVelocity, 3);
      autonomousState = AutonomousState.WAIT_FOR_LAUNCH;

      break;
    case WAIT_FOR_LAUNCH:
      //are we done shooting???
      if (!robot.launcher.isShotRequested()) {
        autonomousState = AutonomousState.MOVE_OUT_OF_ZONE;
        if (alliance == kALLIANCE_BLUE) {
          //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) - 18.5, AngleUnit.DEGREES, 0);
        }
        else {
          //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) + 16, AngleUnit.DEGREES, 0);
        }

      }
      break;
    case MOVE_OUT_OF_ZONE:
      if (!isMoving) {
        if (alliance == kALLIANCE_BLUE) {
          setPathState(PathState.SMALL_BLUE_LAUNCH);
          //robot.move(0, -.5, 0);
        }
        else {
          setPathState(PathState.SMALL_RED_LAUNCH);
          //robot.move(0, .5, 0);
        }
        //start moving to X/Y
        isMoving = true;
      } else {
        //currentPos = robot.DriveTrain().getPinpointPosition();
        //double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
        //double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

        if (follower.atPose(launchSmallBluePose, tolorance, tolorance, tolorance)||follower.atPose(launchSmallRedPose, tolorance, tolorance, tolorance))
        {
          //robot.DriveTrain().stop();
          autonomousState = AutonomousState.COMPLETE;
          //isMoving = false;
        }
        //we are moving, check to see if we get to destination
        }
      break;
    case COMPLETE:
      setPathState(PathState.END);
      //robot.DriveTrain().stop();
      //isMoving = false;
      //finished = true;
      break;
    case STOP:
      robot.DriveTrain().stop();
      isMoving = false;
      //finished = true;
      break;
    }
  }

  //Big/Close Zone
  public void runBasicBigZone() {
    switch (autonomousState) {
      case START:
        //this is to start the spinup
        //currentPos = robot.DriveTrain().getPinpointPosition();
        robot.launcher.target_velocity = kLAUNCHER_TARGET_VELOCITY_CLOSE + bonusVelocity;
        robot.launcher.setFlyWheelVelocity(robot.launcher.target_velocity);
        robot.launcher.setHoodPosition(kHOOD_MIN_POS);
        if (runtime.seconds() > 2) {
          if (alliance == kALLIANCE_BLUE) {
            //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) - 40, AngleUnit.DEGREES, 0);
          }
          else {
            //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) + 40, AngleUnit.DEGREES, 0);
          }
          autonomousState = AutonomousState.MOVE_TO_LAUNCH;
        }
        break;
      case MOVE_TO_LAUNCH:
        if (!isMoving) {
          if ( alliance == kALLIANCE_BLUE) {
            //setPathState(PathState.LARGE_BLUE_START);
            //robot.move(0, -.5, 0);
          }
          else {
            //setPathState(PathState.LARGE_RED_START);
            //robot.move(0, .5, 0);
          }
          //start moving to X/Y
          isMoving = true;
        } else {
          //currentPos = robot.DriveTrain().getPinpointPosition();
          //double dX = currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH);
          //double dY = currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH);
          //double distance = Math.sqrt(dX*dX + dY*dY);
          //if (follower.atPose(launchSmallBluePose, tolorance, tolorance, tolorance))
          //{
            //robot.DriveTrain().stop();
            autonomousState = AutonomousState.LAUNCH;
            //isMoving = false;
          //}
          //we are moving, check to see if we get to destination
        }
        break;
      case LAUNCH:
        //start the shooting process
        robot.launcher.rapidFire = true;
        robot.shoot(true, kLAUNCHER_TARGET_VELOCITY_CLOSE + bonusVelocity, 3);
        autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
        break;
      case WAIT_FOR_LAUNCH:
        //are we done shooting???
        if (!robot.launcher.isShotRequested()) {
          autonomousState = AutonomousState.MOVE_OUT_OF_ZONE;
          if (alliance == kALLIANCE_BLUE) {
            //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) - 26, AngleUnit.DEGREES, 0);
          }
          else {
            //targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,currentPos.getY(DistanceUnit.INCH) + 26, AngleUnit.DEGREES, 0);
          }

        }
        break;
      case MOVE_OUT_OF_ZONE:
        if (!isMoving) {
          if (alliance == kALLIANCE_BLUE) {
            setPathState(PathState.LARGE_BLUE_START);
            //robot.move(0, -.5, 0);
          }
          else {
            setPathState(PathState.LARGE_RED_START);
            //robot.move(0, .5, 0);
          }
          //start moving to X/Y
          isMoving = true;
        } else {
          //currentPos = robot.DriveTrain().getPinpointPosition();
          //double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
          //double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

          if (follower.atPose(endLargeBluePose, tolorance, tolorance, tolorance)||follower.atPose(endLargeRedPose, tolorance, tolorance, tolorance))
          {
            //robot.DriveTrain().stop();
            autonomousState = AutonomousState.COMPLETE;
            //isMoving = false;
          }
          //we are moving, check to see if we get to destination
        }
        break;
      case COMPLETE:
        //robot.DriveTrain().stop();
        //isMoving = false;
        //finished = true;
        break;
      case STOP:
        robot.DriveTrain().stop();
        isMoving = false;
        //finished = true;
        break;
    }
  }

  public void outputPositions(String label, int[] position) {
    telemetry.addData(label, "LF (%d), RF (%d), LB (%d), RB (%d)",
      position[0], position[1], position[2], position[3]);
  }
  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {
    robot.DriveTrain().stop();
  }

  public void telemetryChoice() {
    telemetry.addData("Version", "3");
    if (alliance == kALLIANCE_RED)
      telemetry.addData("Alliance", "kALLIANCE_RED");
    else if (alliance == kALLIANCE_BLUE)
      telemetry.addData("Alliance", "kALLIANCE_BLUE");
    else
      telemetry.addData("Alliance", "kNOT_SET");

    if (start_location == Location.BASIC_SMALL_ZONE) {
        if (bonusAuto) {
            telemetry.addData("Auto To Run", "Basic Small/Far Zone");
        } else {
            telemetry.addData("Auto To Run", "Basic Small/Far Zone w/bonus");
        }
    }
    else if (start_location == Location.BASIC_BIG_ZONE) {
        if (bonusAuto) {
            telemetry.addData("Auto To Run", "Big/Close Zone");
        } else {
            telemetry.addData("Auto To Run", "Big/Close Zone w/bonus");
        }
    }
    else{
        telemetry.addData("Auto To Run", "kNOT_SET");
    }
    telemetry.addLine("D-Pad Up/Down to change bonusVelocity");
    telemetry.addData("Bonus Velocity", bonusVelocity);
    telemetry.update();
  }
}

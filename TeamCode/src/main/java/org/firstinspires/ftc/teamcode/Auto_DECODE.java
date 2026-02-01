package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

@Autonomous(name = "Auto_DECODE")
public class Auto_DECODE extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  int shotsToFire = 3; //The number of shots to fire in this auto.
  double robotRotationAngle = 45;

  private Robot robot;
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
  private boolean isMoving = false;
  private Pose2D currentPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
  private Pose2D targetPos = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
  private int bonusVelocity = 0;
  private boolean bonusAuto = false;
  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    // Hardware map setup
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
      robot.getAprilTag();

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
      robot.launcher.limeLight.vision.start();

  }

  /*
   * Code to run REPEATEDLY after the driver hits START but before they hit STOP
   */
  @Override
  public void loop() {
    robot.update();
    robot.process(runtime);

    if (start_location == Location.BASIC_SMALL_ZONE) {
      runBasicSmallZone();
    } else if (start_location == Location.BASIC_BIG_ZONE) {
      runBasicBigZone();
    }

    telemetry.addData("AutoState", autonomousState);
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
    telemetry.addData("heading", Math.round(robot.DriveTrain().getPinpointPosition().getHeading(AngleUnit.DEGREES)));
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
      currentPos = robot.DriveTrain().getPinpointPosition();
      robot.launcher.target_velocity = kLAUNCHER_TARGET_VELOCITY_FAR + bonusVelocity;
      robot.launcher.setFlyWheelVelocity(robot.launcher.target_velocity);
      robot.launcher.setHoodPosition(kHOOD_MAX_POS);

      if (runtime.seconds() > 2) {
        if (alliance == kALLIANCE_BLUE) {
          targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                  currentPos.getY(DistanceUnit.INCH) - 12.5, AngleUnit.DEGREES, 0);
        }
        else {
          targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                  currentPos.getY(DistanceUnit.INCH) + 12.5, AngleUnit.DEGREES, 0);
        }
        autonomousState = AutonomousState.MOVE_TO_LAUNCH;
      }
      break;
      case MOVE_TO_LAUNCH:
        if (!isMoving) {
          if ( alliance == kALLIANCE_BLUE) {
            robot.move(0, -.5, 0);
          }
          else {
            robot.move(0, .5, 0);
          }
          //start moving to X/Y
          isMoving = true;
        } else {
          currentPos = robot.DriveTrain().getPinpointPosition();
          double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
          double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

          if ((dY < 1))
          {
            robot.DriveTrain().stop();
            autonomousState = AutonomousState.LAUNCH;
            isMoving = false;
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
          targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                  currentPos.getY(DistanceUnit.INCH) - 14, AngleUnit.DEGREES, 0);
        }
        else {
          targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                  currentPos.getY(DistanceUnit.INCH) + 14, AngleUnit.DEGREES, 0);
        }

      }
      break;
    case MOVE_OUT_OF_ZONE:
      if (!isMoving) {
        if (alliance == kALLIANCE_BLUE) {
          robot.move(0, -.5, 0);
        }
        else {
          robot.move(0, .5, 0);
        }
        //start moving to X/Y
        isMoving = true;
      } else {
        currentPos = robot.DriveTrain().getPinpointPosition();
          double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
          double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

        if ((dY < 1))
        {
          robot.DriveTrain().stop();
            isMoving = false;

            if (bonusAuto){
              // continue bounus auto
              autonomousState = AutonomousState.INTAKE_AT_SPIKE;
              // todo turn intake on and move forward
                robot.intake(1, true);
                if (alliance == kALLIANCE_BLUE) {
                    targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) - 24,
                            currentPos.getY(DistanceUnit.INCH) - 0, AngleUnit.DEGREES, 0);
                }
                else {
                    targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 24,
                            currentPos.getY(DistanceUnit.INCH) + 0, AngleUnit.DEGREES, 0);
                }
          }
          else{
              autonomousState = AutonomousState.COMPLETE;

          }
        }
        //we are moving, check to see if we get to destination
        }
      break;
        case  INTAKE_AT_SPIKE:
        {
            if (!isMoving) {
                if (alliance == kALLIANCE_BLUE) {
                    robot.move(-.5, 0, 0);
                }
                else {
                    robot.move(.5, 0, 0);
                }
                //start moving to X/Y
                isMoving = true;
            } else {
                currentPos = robot.DriveTrain().getPinpointPosition();
                double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
                double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

                if ((dX < 1)) {
                    robot.intake(0, true);
                    robot.DriveTrain().stop();
                    isMoving = false;

                    autonomousState = AutonomousState.MOVE_TO_ZONE;
                    if (alliance == kALLIANCE_BLUE) {
                        targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) - 30,
                                currentPos.getY(DistanceUnit.INCH) - 30, AngleUnit.DEGREES, 0);
                    }
                    else {
                        targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 30,
                                currentPos.getY(DistanceUnit.INCH) + 30, AngleUnit.DEGREES, 0);
                    }
                }
            }
            break;
        }
    case MOVE_TO_ZONE:
    {
        if (!isMoving) {
            if (alliance == kALLIANCE_BLUE) {
                robot.move(-.5, -.5, 0);
            }
            else {
                robot.move(.5, 0.5, 0);
            }
            //start moving to X/Y
            isMoving = true;
        } else {
            currentPos = robot.DriveTrain().getPinpointPosition();
            double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
            double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));
            double distance = Math.sqrt(dX*dX + dY*dY);
            if ((dX < 1)) {
                autonomousState = AutonomousState.COMPLETE;
                robot.DriveTrain().stop();
                isMoving = false;
            }
        }
        break;
    }
    case COMPLETE:
      robot.DriveTrain().stop();
      isMoving = false;
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
        currentPos = robot.DriveTrain().getPinpointPosition();
        robot.launcher.target_velocity = kLAUNCHER_TARGET_VELOCITY_CLOSE + bonusVelocity;
        robot.launcher.setFlyWheelVelocity(robot.launcher.target_velocity);
        robot.launcher.setHoodPosition(kHOOD_MIN_POS);
        if (runtime.seconds() > 2) {
          if (alliance == kALLIANCE_BLUE) {
            targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                    currentPos.getY(DistanceUnit.INCH) - 40, AngleUnit.DEGREES, 0);
          }
          else {
            targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                    currentPos.getY(DistanceUnit.INCH) + 40, AngleUnit.DEGREES, 0);
          }
          autonomousState = AutonomousState.MOVE_TO_LAUNCH;
        }
        break;
      case MOVE_TO_LAUNCH:
        if (!isMoving) {
          if ( alliance == kALLIANCE_BLUE) {
            robot.move(0, -.5, 0);
          }
          else {
            robot.move(0, .5, 0);
          }
          //start moving to X/Y
          isMoving = true;
        } else {
          currentPos = robot.DriveTrain().getPinpointPosition();
          double dX = currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH);
          double dY = currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH);
          double distance = Math.sqrt(dX*dX + dY*dY);
          if (Math.abs(dY) < 1)
          {
            robot.DriveTrain().stop();
            autonomousState = AutonomousState.LAUNCH;
            isMoving = false;
          }
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
            targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                    currentPos.getY(DistanceUnit.INCH) - 26, AngleUnit.DEGREES, 0);
          }
          else {
            targetPos = new Pose2D(DistanceUnit.INCH, currentPos.getX(DistanceUnit.INCH) + 0,
                    currentPos.getY(DistanceUnit.INCH) + 26, AngleUnit.DEGREES, 0);
          }

        }
        break;
      case MOVE_OUT_OF_ZONE:
        if (!isMoving) {
          if (alliance == kALLIANCE_BLUE) {
            robot.move(0, -.5, 0);
          }
          else {
            robot.move(0, .5, 0);
          }
          //start moving to X/Y
          isMoving = true;
        } else {
          currentPos = robot.DriveTrain().getPinpointPosition();
          double dX = Math.abs(currentPos.getX(DistanceUnit.INCH) - targetPos.getX(DistanceUnit.INCH));
          double dY = Math.abs(currentPos.getY(DistanceUnit.INCH) - targetPos.getY(DistanceUnit.INCH));

          if ((dY < 1))
          {
            robot.DriveTrain().stop();
            autonomousState = AutonomousState.COMPLETE;
            isMoving = false;
          }
          //we are moving, check to see if we get to destination
        }
        break;
      case COMPLETE:
        robot.DriveTrain().stop();
        isMoving = false;
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
    telemetry.addData("Pipeline", robot.launcher.limeLight.getPipeline());
    telemetry.addData("Version", "3");
    if (alliance == kALLIANCE_RED)
      telemetry.addData("Alliance", "kALLIANCE_RED");
    else if (alliance == kALLIANCE_BLUE)
      telemetry.addData("Alliance", "kALLIANCE_BLUE");
    else
      telemetry.addData("Alliance", "kNOT_SET");

    if (start_location == Location.BASIC_SMALL_ZONE) {
        if (bonusAuto) {
            telemetry.addData("Auto To Run", "Basic Small/Far Zone w/bonus");
        } else {
            telemetry.addData("Auto To Run", "Basic Small/Far Zone");
        }
    }
    else if (start_location == Location.BASIC_BIG_ZONE) {
        if (bonusAuto) {
            telemetry.addData("Auto To Run", "Big/Close Zone w/bonus");
        } else {
            telemetry.addData("Auto To Run", "Big/Close Zone");
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

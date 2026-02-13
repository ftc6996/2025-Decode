package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.*;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.Limelight3A;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight;


public class Launcher {
    private Servo hood_servo;
    public Servo turret_feeder_servo;

    private DcMotor intake_motor;
    private CRServo servo0, servo1, servo2;

    private CRServo turret_intake_servo;
    private CRServo turret_flywheel_servo;
    private CRServo turret_servo_left;
    private CRServo turret_servo_right;
    private DcMotor turret_encoder;
    public TouchSensor turret_left_limit_sensor = null;
    public TouchSensor turret_right_limit_sensor = null;
    public DcMotorEx turret_flywheel_motor;
    ElapsedTime feederTimer = new ElapsedTime();

    public Limelight3A vision;
    public LimeLight limeLight;

    public int target_velocity;
    public int target_tag = kTAG_ANY;
    public boolean isTargetFound = false;
    public boolean isAccurite = false;
    private boolean shotRequested = false;
    public boolean rapidFire = false;
    public Pose2D aprilTagPose =  new Pose2D(DistanceUnit.INCH, kRED_GOAL_X_OFFSET, kRED_GOAL_Y_OFFSET, AngleUnit.DEGREES, 0);
    public double dX, dY;
    public int numShotsRequested = 0;
    public int numShotsFufiled = 0;

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        STOPPED
    }
    public LaunchState launchState;
    public enum TurningState {
        IDLE,
        START_LEFT,
        START_RIGHT,
        READING,
        STOP
    }
    public TurningState turningState;
    public enum KickingState {
        DOWN,
        UP
    }
    public KickingState kickingState;
    public Launcher ()
    {
        turningState = TurningState.IDLE;
        launchState = LaunchState.IDLE;
        kickingState = KickingState.DOWN;
    }

    public void init(HardwareMap hardwareMap)
    {
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo0 = hardwareMap.get(CRServo.class, "intake_servo0");
        servo1 = hardwareMap.get(CRServo.class, "intake_servo1");
        servo2 = hardwareMap.get(CRServo.class, "intake_servo2");

        hood_servo = hardwareMap.get(Servo.class, "turret_hood_servo");

        turret_feeder_servo = hardwareMap.get(Servo.class, "turret_feeder_servo");

        turret_servo_left = hardwareMap.get(CRServo.class, "turret_left_servo");
        turret_servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_servo_right = hardwareMap.get(CRServo.class, "turret_right_servo");
        turret_servo_right.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_encoder = hardwareMap.get(DcMotor.class, "turret_encoder");
        turret_flywheel_motor = hardwareMap.get(DcMotorEx.class, "turret_flywheel_motor");
        turret_flywheel_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        turret_flywheel_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(94, 0,0, 12);
        turret_flywheel_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        turret_left_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_left_limit_sensor");
        turret_right_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_right_limit_sensor");

        turret_feeder_servo.setPosition(kFEED_OPEN_POS);

        vision = hardwareMap.get(Limelight3A.class, "limelight");
        limeLight = new LimeLight(vision);
    }

    public void setIntakeMotor(double power, boolean all)
    {
        intake_motor.setPower(power);
        servo0.setPower(power);
        servo1.setPower(-power);
        if (all)
            servo2.setPower(-power);
    }
    /// Power > 0 turn left, Power < 0 turn right
    public void setTurretPower(double power)
    {
        turret_servo_left.setPower(power);
        turret_servo_right.setPower(power);
        //turningState = Launcher.TurningState.IDLE;
    }

    public void setFlyWheelVelocity(double velocity)
    {
        turret_flywheel_motor.setVelocity(velocity);
    }
    public void setHoodPosition(double pos)
    {
        //pos = Range.clip(pos, kHOOD_MIN_POS, kHOOD_MAX_POS);
        hood_servo.setPosition(pos);
    }

    public double getHoodPositon()
    {
        return hood_servo.getPosition();
    }
    public double getPositon()
    {
        return turret_encoder.getCurrentPosition();
    }
    public double getTurretAngle()
    {
        return getPositon() / kENCODER_CPR;
    }
    public void resetTurret()
    {
        turret_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public boolean isLeftSensorTriggered(){
       return turret_left_limit_sensor.isPressed();
    }
    public boolean isRightSensorTriggered(){
      return turret_right_limit_sensor.isPressed();
    }
    public void process(Pose2D position)
    {
        limeLight.clearFoundAprilTag();
        limeLight.getAprilTags();
        isTargetFound = (limeLight.getTagID() == target_tag);
        //isAccurite = false;

        Pose2D turretCenter = new Pose2D(DistanceUnit.INCH,
                position.getX(DistanceUnit.INCH) - (5 * Math.cos(position.getHeading(AngleUnit.RADIANS))),
                position.getY(DistanceUnit.INCH) - (2.8 * Math.sin(position.getHeading(AngleUnit.RADIANS))),
                AngleUnit.RADIANS,
                position.getHeading(AngleUnit.RADIANS)
        );

        //always getting the distance from the robot to the april tag
        dX = turretCenter.getX(DistanceUnit.INCH) - aprilTagPose.getX(DistanceUnit.INCH);
        dY = turretCenter.getY(DistanceUnit.INCH) - aprilTagPose.getY(DistanceUnit.INCH);

        switch (turningState)
        {
            case IDLE:
            {
                //do nothing
                break;
            }
            case START_LEFT:
            {
                setTurretPower(-kAUTO_TURN_SPEED);
                turningState = TurningState.READING;
                break;
            }
            case START_RIGHT:
            {
                setTurretPower(kAUTO_TURN_SPEED);
                turningState = TurningState.READING;
                break;
            }
            case READING:
            {
                if (isTargetFound) {
                   // if (Math.abs(limeLight.getTagLocationX()) < 15)
                    //{
                        isAccurite = true;
                        turningState = TurningState.STOP;
                   // }
                }

                //check to see if we can read the tag, maybe some tolerance
                break;
            }
            case STOP:
            {
                turningState = TurningState.IDLE;
                setTurretPower(0.0);
                break;
            }
            default:
            {
                //do nothing
                break;
            }
        }
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    turret_feeder_servo.setPosition(kFEED_OPEN_POS);
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                turret_flywheel_motor.setVelocity(target_velocity);
/*
                if (target_velocity == kLAUNCHER_TARGET_VELOCITY_FAR)
                {
                    hood_servo.setPosition(kHOOD_MAX_POS);
                }
                else
                {
                   hood_servo.setPosition(kHOOD_MIN_POS);
                }
*/

                double current_velocity = getFlyWheelVelocity();
                if (current_velocity > (target_velocity * 1.00)) {//used to be 1.00
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //turret_feeder_servo.setPosition(kFEED_OPEN_POS);
                feederTimer.reset();
                if (!rapidFire) {
                    intake_motor.setPower(0);
                    servo0.setPower(0);
                    servo1.setPower(0);
                    servo2.setPower(-1);
                    turret_feeder_servo.setPosition(kFEED_CLOSE_POS);
                    numShotsFufiled += 1;
                }else{
                    setIntakeMotor(.8, true);
                }
                launchState = LaunchState.LAUNCHING;
                break;
                //speed 1200, hood 5, passthrough 0.8, distance 3020.5mm, rapidFire.
            case LAUNCHING:
                if(!rapidFire) {
                    if (feederTimer.seconds() > kFEED_TIME_SECONDS) {
                        //if (numShotsFufiled <= numShotsRequested){
                        //    launchState = LaunchState.SPIN_UP;
                        //}else{
                            turret_feeder_servo.setPosition(kFEED_OPEN_POS);
                            setIntakeMotor(0, true);
                            numShotsFufiled = 0;
                            numShotsRequested = 0;

                            launchState = LaunchState.IDLE;
                            shotRequested = false;
                       // }
                    }
                }else{
                    turret_flywheel_motor.setVelocity(target_velocity);
                    if (feederTimer.seconds() > 3){
                        launchState = LaunchState.IDLE;
                        turret_feeder_servo.setPosition(kFEED_OPEN_POS);
                        //intake_motor.setPower(0);
                        //servo0.setPower(0);
                        //servo1.setPower(0);
                        //servo2.setPower(0);
                        shotRequested = false;
                        rapidFire = false;
                    }else if (feederTimer.seconds() > 2) {
                        turret_feeder_servo.setPosition(kFEED_CLOSE_POS);
                    }
                }
                break;
        }
    }

    public double getFlyWheelVelocity()
    {
        return Math.abs(turret_flywheel_motor.getVelocity());
    }
    public double getTargetVelocity()
    {
        return target_velocity;
    }
    public void shoot(boolean startShoot, int velocity, int numShots)
    {
        numShotsRequested = numShots;
        shotRequested = startShoot;
        target_velocity = Math.abs(velocity);
    }
    public boolean isShotRequested()
    {
        return shotRequested;
    }
}

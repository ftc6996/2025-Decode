package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.Launcher.kFEED_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kFEED_OPEN_POS;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kFEED_TIME_SECONDS;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private Servo hood_servo;
    private Servo turret_feeder_servo;

    private CRServo turret_intake_servo;
    private CRServo turret_flywheel_servo;
    private CRServo turret_servo_left;
    private CRServo turret_servo_right;
    private DcMotor turret_encoder;
    public TouchSensor turret_left_limit_sensor = null;
    public TouchSensor turret_right_limit_sensor = null;
    private DcMotorEx turret_flywheel_motor;
    ElapsedTime feederTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        STOPPED
    }
    private LaunchState launchState;

    public Launcher ()
    {
        launchState = LaunchState.IDLE;
    }


    public void init(HardwareMap hardwareMap)
    {
        hood_servo = hardwareMap.get(Servo.class, "turret_hood_servo");

        turret_feeder_servo = hardwareMap.get(Servo.class, "turret_feeder_servo");

        turret_servo_left = hardwareMap.get(CRServo.class, "turret_left_servo");
        turret_servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_servo_right = hardwareMap.get(CRServo.class, "turret_right_servo");
        turret_servo_right.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_encoder = hardwareMap.get(DcMotor.class, "turret_encoder");
        turret_flywheel_motor = hardwareMap.get(DcMotorEx.class, "turret_flywheel_motor");

        turret_left_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_left_limit_sensor");
        turret_right_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_right_limit_sensor");
    }

    /// Power > 0 turn left, Power < 0 turn right
    public void setTurretPower(double power)
    {
        turret_servo_left.setPower(power);
        turret_servo_right.setPower(power);
    }

    public void setHoodPosition(double pos)
    {
        pos = Range.clip(pos, Constants.Launcher.kHOOD_MIN_POS,Constants.Launcher.kHOOD_MAX_POS);
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
    public boolean isLeftSensorTriggered(){
       return turret_left_limit_sensor.isPressed();
    }
    public boolean isRightSensorTriggered(){
      return turret_right_limit_sensor.isPressed();
    }
    public void process()
    {
        /*
        if (isLeftSensorTriggered())
        {
            turret_encoder.setMode(DcMotor.RunMode.RESET_ENCODERS);
        }
         */
    }

    public void shoot(boolean shotRequested, int velocity)
    {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                turret_flywheel_motor.setVelocity(velocity);
                if (turret_flywheel_motor.getVelocity() > (velocity * .80)) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                turret_feeder_servo.setPosition(kFEED_CLOSE_POS);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > kFEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    turret_feeder_servo.setPosition(kFEED_OPEN_POS);
                }
                break;
        }
    }
/*
    private void rotateToTag(boolean turnRequested)
    {
        switch(turningState)
        {
            case IDLE:{
                if (turnRequested) {
                    turningState = TurningState.START;
                }
                break;
            }
            case START:{
                arcadeDrive(0, .25);
                turningState = TurningState.READING;
                break;

            }
            case READING:{
                if (targetFound)
                {
                    turningState = TurningState.STOP;
                }
                break;

            }
            case STOP:{
                arcadeDrive(0, 0);
                turningState = TurningState.IDLE;
                launch(true);
                break;

            }
        }
    }
    */

}

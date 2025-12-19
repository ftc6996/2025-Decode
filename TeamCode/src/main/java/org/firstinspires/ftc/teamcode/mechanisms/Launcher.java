package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private Servo hood_servo;
    private CRServo turret_servo_left;
    private CRServo turret_servo_right;
    private DcMotor turret_encoder;
    public TouchSensor turret_left_limit_sensor = null;
    public TouchSensor turret_right_limit_sensor = null;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;
    public Launcher ()
    {

    }


    public void init(HardwareMap hardwareMap)
    {
        hood_servo = hardwareMap.get(Servo.class, "turret_hood_servo");

        turret_servo_left = hardwareMap.get(CRServo.class, "turret_left_servo");
        turret_servo_left.setDirection(DcMotorSimple.Direction.REVERSE);

        turret_servo_right = hardwareMap.get(CRServo.class, "turret_right_servo");
        turret_servo_right.setDirection(DcMotorSimple.Direction.REVERSE);

        //turret_encoder = hardwareMap.get(DcMotor.class, "turret_encoder");

        turret_left_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_left_limit_sensor");
        turret_right_limit_sensor = hardwareMap.get(TouchSensor.class, "turret_right_limit_sensor");
    }

    public void setTurretPower(double power)
    {
        if (!isLeftSensorTriggered() && !isRightSensorTriggered()){
        turret_servo_left.setPower(power);
        turret_servo_right.setPower(power);}

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

    }

}

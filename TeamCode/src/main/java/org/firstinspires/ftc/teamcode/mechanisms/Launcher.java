package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants;

public class Launcher {
    private Servo hood_servo;
    private CRServo turret_servo;
    public Launcher ()
    {}


    public void init(HardwareMap hardwareMap)
    {
        hood_servo = hardwareMap.get(Servo.class, "hood_servo");
        turret_servo = hardwareMap.get(CRServo.class, "turret_servo");
    }

    public void setTurretPower(double power)
    {
        turret_servo.setPower(power);
    }
    public void setHoodPosition(double pos)
    {
        pos = Range.clip(pos, Constants.Launcher.kHOOD_MIN_POS,Constants.Launcher.kHOOD_MAX_POS);
        hood_servo.setPosition(pos);
    }

    public double getPositon()
    {
        return hood_servo.getPosition();
    }
}

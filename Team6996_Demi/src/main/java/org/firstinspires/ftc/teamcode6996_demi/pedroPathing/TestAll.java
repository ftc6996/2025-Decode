package org.firstinspires.ftc.teamcode6996_demi.pedroPathing;

import static android.os.SystemClock.sleep;

import org.firstinspires.ftc.teamcode6996_demi.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TestAll", group="TeleOp")
public class TestAll extends OpMode {
    Servo turret_hood_servo;
    CRServo turret_left_servo;
    CRServo turret_right_servo;
    @Override
    public void init(){
        turret_hood_servo = hardwareMap.get(Servo.class, "turret_hood_servo");
        turret_left_servo = hardwareMap.get(CRServo.class, "turret_left_servo");
        turret_left_servo.setDirection(DcMotorSimple.Direction.REVERSE);
        turret_right_servo = hardwareMap.get(CRServo.class, "turret_right_servo");
        turret_right_servo.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop(){
        turret_left_servo.setPower(1.0);
        turret_right_servo.setPower(1.0);
    }

    @Override
    public void start() {

        //turret_right_servo.setPosition(1);

    }
}

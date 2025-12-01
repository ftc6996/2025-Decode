package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Constants.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="TestServo", group="Test")
public class TestServo extends OpMode {

    public CRServo turret_servo;
    public Servo hood_servo;
    @Override
    public void init() {
        turret_servo = hardwareMap.get(CRServo.class, "turret_servo");
       // hood_servo = hardwareMap.get(Servo.class, "hood_servo");
    }


    @Override
    public void init_loop() {

    }
    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if (gamepad1.xWasPressed())
        {
           // hood_servo.setPosition(.7);
        }
        if (gamepad1.yWasPressed())
        {
//hood_servo.setPosition(.1);
        }
        if (gamepad1.right_trigger > 0)
        {
            //turret_servo.setPower(gamepad1.right_trigger);
            turret_servo.setPower(1);
        }
        else if (gamepad1.left_trigger > 0)
        {
            //turret_servo.setPower(-gamepad1.left_trigger);
            turret_servo.setPower(-1);
        }
        else
        {
            turret_servo.setPower(0);
        }
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.update();
    }
}

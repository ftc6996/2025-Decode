package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTest extends OpMode {

    public DcMotorEx turret_flywheel_motor;
    public double highVelocity = 1500;
    public double lowVelocity = 800;
    public double currentTargetVelocity = highVelocity;
    public double F = 0;
    public double P = 0;
    public double[] stepSize = {10.0, 1.0, 0.1, 0.001, 0.0001};
    public int stepSizeIndex = 1;

    @Override
    public void init()
    {
        turret_flywheel_motor = hardwareMap.get(DcMotorEx.class, "turret_flywheel_motor");
        turret_flywheel_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        turret_flywheel_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0,0, F);
        turret_flywheel_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop()
    {

        //switch between high and low velocity
        if (gamepad1.yWasPressed())
        {
            if (currentTargetVelocity == highVelocity)
                currentTargetVelocity = lowVelocity;
            else currentTargetVelocity = highVelocity;
        }

        //step through the step sizes
        if (gamepad1.bWasPressed())
        {
            stepSizeIndex = (stepSizeIndex + 1) % stepSize.length;
        }

        if (gamepad1.dpadLeftWasPressed())
        {
            F -= stepSize[stepSizeIndex];
        }
        if (gamepad1.dpadRightWasPressed())
        {
            F += stepSize[stepSizeIndex];
        }

        if (gamepad1.dpadUpWasPressed())
        {
            P += stepSize[stepSizeIndex];
        }
        if (gamepad1.dpadDownWasPressed())
        {
            P -= stepSize[stepSizeIndex];
        }

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0,0, F);
        turret_flywheel_motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        turret_flywheel_motor.setVelocity(currentTargetVelocity);

        double currentVelocity = turret_flywheel_motor.getVelocity();
        double error = currentTargetVelocity - currentVelocity;

        telemetry.addData(" Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", currentVelocity);
        telemetry.addData("           Error", "%.2f", error);
        telemetry.addLine("------------------------------------");
        telemetry.addData(" Tuning P", "%.4f (U/D)", P);
        telemetry.addData(" Tuning F", "%.4f (L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSize[stepSizeIndex]);

    }

}

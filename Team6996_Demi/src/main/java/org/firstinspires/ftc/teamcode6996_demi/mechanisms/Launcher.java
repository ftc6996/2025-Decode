package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode6996_demi.Auto_DECODE;

public class Launcher {

    public enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }
    private LaunchState launchState = LaunchState.IDLE;

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();

    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double TIME_BETWEEN_SHOTS = 2;
    final double FEED_TIME = 0.20;

    private double target_velocity = 1125;
    private double min_velocity = 1025;
    private boolean is_initialized = false;
    //declare motors or servos
    private DcMotorEx launchMotor = null;
    private CRServo left_feeder, right_feeder;
    public void init(HardwareMap hardwareMap) {
        //init hardware
        launchMotor = hardwareMap.get(DcMotorEx.class, "launch_motor");
        if (launchMotor != null)
        {
            launchMotor.setDirection(DcMotor.Direction.FORWARD);
            launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));
            is_initialized = true;
        }
        left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
    }

    public LaunchState getLaunchState()
    {
        return launchState;
    }
    public void setMinVelocity(double velocity)
    {
        min_velocity = velocity;
    }

    public void setVelocity(double velocity)
    {
        target_velocity = velocity;
        if (is_initialized)
        {
            launchMotor.setVelocity(target_velocity);
        }
    }

    public double getVelocity()
    {
        double velocity = 0;
        if (is_initialized)
        {
            velocity = launchMotor.getVelocity();
        }
        return velocity;
    }
    public boolean hasReachedMinVelocity()
    {
        boolean success = false;
        if (getVelocity() > min_velocity)
        {
            success = true;
        }
        return success;
    }

    public void enableFeeder(boolean enabled)
    {
        if (is_initialized)
        {
            if (enabled)
            {
                left_feeder.setPower(1);
                right_feeder.setPower(1);
            }
            else
            {
                left_feeder.setPower(0);
                right_feeder.setPower(0);
            }
        }//hardware initialized
    }

    public boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (hasReachedMinVelocity())
                {
                    launchState = LaunchState.LAUNCH;
                    enableFeeder(true);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME)
                {
                    enableFeeder(false);
                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }
}
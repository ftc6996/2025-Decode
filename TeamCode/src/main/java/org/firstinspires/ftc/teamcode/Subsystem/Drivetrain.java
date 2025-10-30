package org.firstinspires.ftc.teamcode.Subsystem;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants;

import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {

    private MecanumDrive mecanum;
    private MotorEx [] motors;
    static final public int Version = 1;
    public Drivetrain(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight)
    {
        this.motors = new MotorEx[Constants.kMAX_DRIVE_MOTORS];
        this.motors[Constants.kLEFT_FRONT] = frontLeft;
        this.motors[Constants.kRIGHT_FRONT] = frontRight;
        this.motors[Constants.kLEFT_REAR] = backLeft;
        this.motors[Constants.kRIGHT_REAR] = backRight;

        setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.motors[Constants.kLEFT_FRONT].setInverted(true);
        this.motors[Constants.kRIGHT_FRONT].setInverted(true);
        this.motors[Constants.kLEFT_REAR].setInverted(true);
        this.motors[Constants.kRIGHT_REAR].setInverted(true);

        mecanum = new MecanumDrive(
                this.motors[Constants.kLEFT_FRONT] , this.motors[Constants.kRIGHT_FRONT],
                this.motors[Constants.kLEFT_REAR] , this.motors[Constants.kRIGHT_REAR]);
    }

    @Override
    public void periodic()
    {
        // This method will be called once per scheduler run
    }

    public void drive(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn)
    {
        mecanum.driveRobotCentric(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble());
    }

    public void driveMotorPower(Double frontLeft, Double frontRight, Double backLeft, Double backRight)
    {
        mecanum.driveWithMotorPowers(frontLeft, frontRight, backLeft, backRight);
    }

    public void driveFieldOriented(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, Double heading)
    {
        mecanum.driveFieldCentric(strafe.getAsDouble(), forward.getAsDouble(), turn.getAsDouble(), heading);
    }

    public void setZeroPowerBehavior(Motor.ZeroPowerBehavior mode)
    {
        for(MotorEx m : motors)
        {
            m.setZeroPowerBehavior(mode);
        }
    }

    public void setZeroPowerBehavior(int index, Motor.ZeroPowerBehavior mode)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "setZeroPowerBehavior: invalid index");
            return;
        }
        this.motors[index].setZeroPowerBehavior(mode);
    }

    public double getAcceleration(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getAcceleration: invalid index");
            return 0;
        }
        return this.motors[index].getAcceleration();
    }

    public double [] getAcceleration()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.getAcceleration();
        }
        return arr;
    }

    public double getPower(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getPower: invalid index");
            return 0;
        }
        return this.motors[index].get();
    }

    public double [] getPower()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.get();
        }
        return arr;
    }

    public double getVelocity(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getVelocity: invalid index");
            return 0;
        }
        return this.motors[index].getVelocity();
    }

    public double [] getVelocity()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.getVelocity();
        }
        return arr;
    }

    public double getRate(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getRate: invalid index");
            return 0;
        }
        return this.motors[index].getRate();
    }

    public double [] getRate()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.getRate();
        }
        return arr;
    }

    public double getCPR(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getCPR: invalid index");
            return 0;
        }
        return this.motors[index].getCPR();
    }

    public double [] getCPR()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.getCPR();
        }
        return arr;
    }

    public double getDistance(int index)
    {
        if ((index < 0) || (index >= Constants.kMAX_DRIVE_MOTORS))
        {
            Log.d("Drivetrain", "getDistance: invalid index");
            return 0;
        }
        return this.motors[index].getDistance();
    }

    public double [] getDistance()
    {
        double[] arr = new double[Constants.kMAX_DRIVE_MOTORS];
        int idx = 0;
        for(MotorEx m : motors)
        {
            arr[idx++]= m.getDistance();
        }
        return arr;
    }
}

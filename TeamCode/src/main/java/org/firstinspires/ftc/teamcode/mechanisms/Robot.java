package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
public class Robot {

    private MecanumDrive mecanumDrive;
    private DcMotor intake_motor;
    private Blinky blinky;
    private Servo rgb_light;
    private Launcher launcher;
    private CRServo servo0, servo1, servo2;

    private int alliance = kNOT_SET;

    public Robot()
    {
    }
    public void init(HardwareMap hardwareMap) {

        mecanumDrive = new MecanumDrive(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        mecanumDrive.init(hardwareMap);

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.FORWARD);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo0 = hardwareMap.get(CRServo.class, "intake_servo0");
        servo1 = hardwareMap.get(CRServo.class, "intake_servo1");
        servo2 = hardwareMap.get(CRServo.class, "intake_servo2");

        blinky = new Blinky();
        blinky.init(hardwareMap);

        rgb_light = hardwareMap.get(Servo.class, "rgb_light");
        rgb_light.setPosition(0);
        launcher = new Launcher();
        launcher.init(hardwareMap);
    }

    public void update()
    {
        mecanumDrive.updatePinPoint();
    }
    public void process(ElapsedTime current)
    {
        blinky.process(current);
        launcher.process();
    }

    public void changeSpeed(double speed)
    {
        double current_speed = mecanumDrive.getMaxSpeed();
        mecanumDrive.setMaxSpeed(current_speed + speed);
    }
    public MecanumDrive DriveTrain(){ return mecanumDrive; }
    public void setMaxSpeed(double speed)
    {
        mecanumDrive.setMaxSpeed(speed);
    }
    public double getMaxSpeed()
    {
        return mecanumDrive.getMaxSpeed();
    }
    public void move(double drive, double strafe, double twist)
    {
        mecanumDrive.move(drive, strafe, twist);
    }
    public void intake(double power)
    {
        intake_motor.setPower(power);
        servo0.setPower(power);
        servo1.setPower(-power);
        servo2.setPower(-power);
    }

    public void seekTagLeft()
    {
        //set some value for state machine
        launcher.turningState = Launcher.TurningState.START_LEFT;
    }
    public void seekTagRight()
    {
        //set some value for state machine
        launcher.turningState = Launcher.TurningState.START_RIGHT;
    }
    public void setAlliance(int alliance)
    {
        if (alliance == kALLIANCE_RED)
        {
            blinky.setRedAlliance();
            rgb_light.setPosition(.28);
            launcher.limeLight.setPipeline(Game.kPIPELINE_ALLIANCE_RED);
        }
        else if (alliance == kALLIANCE_BLUE)
        {
            launcher.limeLight.setPipeline(Game.kPIPELINE_ALLIANCE_BLUE);
            blinky.setBlueAlliance();
            rgb_light.setPosition(0.611);
        }
        else
        {
            blinky.setUnknownAlliance();
            rgb_light.setPosition(0);
        }
    }

    public void setTurretPower(double power)
    {
        launcher.setTurretPower(power);
    }

    public void setTurretFlyWheelVelocity(double velocity)
    {
        launcher.setFlyWheelVelocity(velocity);
    }
    public void setHoodPosition(double pos)
    {
        launcher.setHoodPosition(pos);
    }
    public double getLauncherHoodPosition()
    {
        return launcher.getHoodPositon();
    }
    public void shoot(boolean shotRequested, int velocity)
    {
        launcher.shoot(shotRequested, velocity);
    }
    public void getAprilTag()
    {
        launcher.limeLight.getAprilTags();
    }
    public void processTelemetry(Telemetry telemetry)
    {
        telemetry.addData("Launch State: ", launcher.launchState);
        telemetry.addData("Launch Target Velocity:", launcher.getTargetVelocity());
        telemetry.addData("Launch Current Velocity:", launcher.getFlyWheelVelocity());
        /*
        telemetry.addData("Speed%: ", current_speed);
        telemetry.addData("heading (degrees)", heading_deg);
        telemetry.addData("Left Joy X", gamepad1.left_stick_x);
        telemetry.addData("Left Joy Y", gamepad1.left_stick_y);
        telemetry.addData("Driver Mode", driver_mode_string);
        int [] target = robot.getAllPositions();
        telemetry.addData("Path", "Driving");
        telemetry.addData("LF", target[0]);
        telemetry.addData("RF", target[1]);
        telemetry.addData("LB", target[2]);
        telemetry.addData("RB", target[3]);
        telemetry.addData("Intake", intake_status);
        telemetry.addData("positionX", Math.round(robot.getPinpointPosition().getX(DistanceUnit.MM)));
        telemetry.addData("positionY", Math.round(robot.getPinpointPosition().getY(DistanceUnit.MM)));
        telemetry.addData("heading",Math.round(robot.getPinpointPosition().getHeading(AngleUnit.DEGREES)));
        telemetry.update();
         */
    }

    public int getTagID() {return launcher.limeLight.getTagID();}

    public double getTagLocationX()
    {
        return launcher.limeLight.getTagLocationX();
    }

    public double getTagLocationY()
    {
        return launcher.limeLight.getTagLocationY();
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    private MecanumDrive mecanumDrive;
    private DcMotor intake_motor;
    private Blinky blinky;
    //private Launcher launcher;
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

        blinky = new Blinky();
        blinky.init(hardwareMap);
    }

    public void update()
    {
        mecanumDrive.updatePinPoint();
    }
    public void changeSpeed(double speed)
    {
        double current_speed = mecanumDrive.getMaxSpeed();
        mecanumDrive.setMaxSpeed(current_speed + speed);
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
    }

    public void setAlliance(int alliance)
    {
        if (alliance == kALLIANCE_RED)
        {
            blinky.setRedAlliance();
        }
        else if (alliance == kALLIANCE_BLUE)
        {
            blinky.setBlueAlliance();
        }
        else
        {
            blinky.setUnknownAlliance();
        }
    }

    public void processTelemetry(Telemetry telemetry)
    {
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
}

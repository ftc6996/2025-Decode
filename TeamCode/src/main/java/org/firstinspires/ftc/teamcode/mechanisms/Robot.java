package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.Constants.*;
import static org.firstinspires.ftc.teamcode.Constants.Game.*;
import static org.firstinspires.ftc.teamcode.Constants.RGB_SERVO_LIGHT.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    private MecanumDrive mecanumDrive;
    public DcMotor intake_motor;
    private Blinky blinky;
    private Servo rgb_light;
    public Launcher launcher;

    private int team_alliance = kNOT_SET;

    ElapsedTime intakeTimer = new ElapsedTime();

    public enum IntakeTime {
        IDLE,
        INTAKEON,
        INTAKEOFF,
        INTAKEONREVERSE
    }
    public IntakeTime intakeTime = IntakeTime.IDLE;

    public Robot() {

    }
    public void init(HardwareMap hardwareMap) {

        mecanumDrive = new MecanumDrive(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        mecanumDrive.init(hardwareMap);

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        blinky = new Blinky();
        blinky.init(hardwareMap);

        rgb_light = hardwareMap.get(Servo.class, "rgb_light");
        rgb_light.setPosition(kBLACK);
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
        if (launcher.isTargetFound)
        {
            rgb_light.setPosition(kGREEN);
        }
        else
        {
            rgb_light.setPosition(kYELLOW);
        }
        switch (intakeTime)
        {
            case IDLE:
            {
                break;
            }
            case INTAKEON:
                intake(1);
                intakeTimer.reset();
                break;
            case INTAKEOFF:
                if(intakeTimer.seconds() >= 1){
                    intake(0);
                }
                break;
            case INTAKEONREVERSE:
                intake(-1);

            default:
                //nothing
                break;
        }
    }

    public void changeSpeed(double speed) {
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
    public void move(double drive, double strafe, double twist) {
        mecanumDrive.move(drive, strafe, twist);
    }
    public void intake(double power) {
        intake_motor.setPower(power);
        launcher.setIntakeMotor(power);
    }

    public void seekTagLeft() {
        //set some value for state machine
        launcher.turningState = Launcher.TurningState.START_LEFT;
    }
    public void seekTagRight() {
        //set some value for state machine
        launcher.turningState = Launcher.TurningState.START_RIGHT;
    }
    public void setAlliance(int alliance) {
        if (alliance == kALLIANCE_RED)
        {
            team_alliance = alliance;
            blinky.setRedAlliance();
            rgb_light.setPosition(kRED);
            launcher.limeLight.setPipeline(Game.kPIPELINE_ALLIANCE_RED);
            launcher.target_tag = kTAG_GOAL_RED;
        }
        else if (alliance == kALLIANCE_BLUE)
        {
            team_alliance = alliance;
            launcher.limeLight.setPipeline(Game.kPIPELINE_ALLIANCE_BLUE);
            launcher.target_tag = kTAG_GOAL_BLUE;
            blinky.setBlueAlliance();
            rgb_light.setPosition(kBLUE);
        }
        else
        {
            team_alliance = kNOT_SET;
            blinky.setUnknownAlliance();
            rgb_light.setPosition(kBLACK);
        }
    }

    public void setTurretPower(double power)
    {
        launcher.setTurretPower(power);
    }
    public void setTurretFlyWheelVelocity(double velocity) {
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
    public void shoot(boolean shotRequested, int velocity, int numShots) {
        launcher.shoot(shotRequested, velocity, numShots);
    }
    public void setKickerDown() {

    }
    public void setKickerUp() {

    }
    public Launcher.KickingState getKickerState()
    {
        return launcher.kickingState;
    }
    public void getAprilTag()
    {
        launcher.limeLight.getAprilTags();
    }
    public void processTelemetry(Telemetry telemetry) {
        telemetry.addData("Launch State: ", launcher.launchState);
        telemetry.addData("Launch Target Velocity:", launcher.getTargetVelocity());
        telemetry.addData("Launch Current Velocity:", launcher.getFlyWheelVelocity());

        telemetry.addLine("-----------------------------------");
        telemetry.addData("Hood Pos",launcher.getHoodPositon());
        telemetry.addData("Left Mag",launcher.isLeftSensorTriggered());
        telemetry.addData("Right Mag",launcher.isRightSensorTriggered());
        telemetry.addData("Turret Pos",launcher.getPositon());
        telemetry.addLine("-----------------------------------");
        telemetry.addData("Pipeline",launcher.limeLight.getPipeline());
        telemetry.addData("turret state",launcher.turningState);

        telemetry.addData("Target Found",launcher.isTargetFound);
        if (launcher.isTargetFound) {
            telemetry.addData("ID seen", launcher.limeLight.getTagID());
            telemetry.addData("Distance", launcher.limeLight.getTagDistance());
            telemetry.addData("Pose", launcher.limeLight.tagPose.getPosition().toUnit(DistanceUnit.INCH).toString());
        }
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

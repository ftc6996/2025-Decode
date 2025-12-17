package org.firstinspires.ftc.teamcode6996_demi.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static void initOthers(HardwareMap hardwareMap){
        /*
        launch_motor = hardwareMap.get(DcMotor.class, "launch_motor");
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");

        Servo turret_hood_servo = hardwareMap.get(Servo.class, "turret_hood_servo");
        Servo turret_left_servo = hardwareMap.get(Servo.class, "turret_left_servo");
        turret_left_servo.setDirection(Servo.Direction.REVERSE);
        Servo turret_right_servo = hardwareMap.get(Servo.class, "turret_right_servo");

        turret_feeder_servo = hardwareMap.get(Servo.class, "turret_feeder_servo");
        park_lift_servo = hardwareMap.get(Servo.class, "park_lift_servo");
        rgb_servo = hardwareMap.get(RevBlinkinLedDriver.class, "rgb_servo");

        turret_encoder = hardwareMap.get(DcMotor.class, "turret_encoder");
        turret_left_limit_sensor = hardwareMap.get(DigitalChannel.class, "turret_left_limit_sensor");
        turret_right_limit_sensor = hardwareMap.get(DigitalChannel.class, "turret_left_limit_sensor");
        left_light_sensor = hardwareMap.get(ColorSensor.class, "left_light_sensor");
        right_light_sensor = hardwareMap.get(ColorSensor.class, "right_light_sensor");
        back_light_sensor = hardwareMap.get(ColorSensor.class, "back_light_sensor");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        turret_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_left_limit_sensor.setMode(DigitalChannel.Mode.INPUT);
        turret_right_limit_sensor.setMode(DigitalChannel.Mode.INPUT);
        left_light_sensor.enableLed(true); // Turn on the white LED if true
        right_light_sensor.enableLed(true); // Turn on the white LED if true
        back_light_sensor.enableLed(true); // Turn on the white LED if true
         */
    }
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5)//must be in kilograms
            .forwardZeroPowerAcceleration(-41.69401197278003)
            .lateralZeroPowerAcceleration(-59.633958674132884)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.0225,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.25,0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0005,0,0.0005,0.6,0.5))
            .centripetalScaling(0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_rear_drive")
            .leftRearMotorName("left_rear_drive")
            .leftFrontMotorName("left_front_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(30.881249134934798)
            .yVelocity(28.502984474963093);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)//0
            .strafePodX(3.54331)//0.125
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}

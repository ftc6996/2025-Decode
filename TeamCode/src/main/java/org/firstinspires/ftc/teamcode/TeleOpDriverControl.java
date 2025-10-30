package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.Drive;
import org.firstinspires.ftc.teamcode.Subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystem.TelemetrySubsystem;

@TeleOp(name = "BasicTeleOp")
public class TeleOpDriverControl extends CommandOpMode {

    private TelemetrySubsystem output;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().run();

        GamepadEx controller1 = new GamepadEx(gamepad1);
        //GamepadEx controller2 = new GamepadEx(gamepad2);

        // Initialize DriveTrain
        Drivetrain drivetrain = new Drivetrain(
                new MotorEx(hardwareMap, "left_front_drive"),
                new MotorEx(hardwareMap, "right_front_drive"),
                new MotorEx(hardwareMap, "left_rear_drive"),
                new MotorEx(hardwareMap, "right_rear_drive")
        );

        output = new TelemetrySubsystem(drivetrain, telemetry);

        drivetrain.setDefaultCommand(new Drive(drivetrain,
                () -> controller1.getLeftY(),       //forward
                () -> controller1.getLeftX(),       //strafe
                () -> controller1.getRightX()));    //turn

        register(
                output
        );
    }// end of initialize
}

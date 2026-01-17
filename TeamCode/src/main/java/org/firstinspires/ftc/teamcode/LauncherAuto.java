package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.Launcher.kLAUNCHER_TARGET_VELOCITY_CLOSE;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LauncherAuto extends OpMode{

    public Robot robot = new Robot();
    public Launcher launcher = new Launcher();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        // Define and Initialize Motors
        robot.init(hardwareMap);

        robot.setKickerDown();
        robot.intakeTime = Robot.IntakeTime.INTAKEOFF;
    }
    @Override
    public void init_loop() {
        telemetry.addData("Version", "1");
        telemetry.update();
    }
    @Override
    public void start() {
        runtime.reset();
        runtime.startTime();

        robot.move(10,0,0);
        launcher.turningState = launcher.turningState.START_LEFT;
        if(launcher.turningState == launcher.turningState.IDLE) {
            robot.shoot(true, 1000);
            if(launcher.launchState == Launcher.LaunchState.IDLE){
                robot.intakeTime = Robot.IntakeTime.INTAKEON;
                robot.intakeTime = Robot.IntakeTime.INTAKEOFF;
                robot.shoot(true, 1000);
                if(launcher.launchState == Launcher.LaunchState.IDLE){
                    robot.intakeTime = Robot.IntakeTime.INTAKEON;
                    robot.intakeTime = Robot.IntakeTime.INTAKEOFF;
                    robot.shoot(true, 1000);
                    if(launcher.launchState == Launcher.LaunchState.IDLE){
                        telemetry.addData("auto done?", "YES!");
                    }
                }
            }
        }
    }
    @Override
    public void loop() {

    }
    @Override
    public void stop() {
        robot.DriveTrain().stop();
    }

}
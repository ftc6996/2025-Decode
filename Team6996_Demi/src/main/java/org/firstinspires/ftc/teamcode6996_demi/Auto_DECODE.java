package org.firstinspires.ftc.teamcode6996_demi;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto_DECODE")
public class Auto_DECODE extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private static final int kNOT_SET = -1;

    private static final int kSTART_LOCATION_1 = 0;
    private static final int kSTART_LOCATION_2 = 1;
    private static final int kSTART_LOCATION_3 = 2;
    public int current_start_location = kNOT_SET;

    private static final int kBLUE = 0;
    private static final int kRED = 1;
    public int alliance_color = kNOT_SET;

    public Gamepad saved_gamepad1 = new Gamepad();
    public String alliance = "blue";
    public String audience = "none";
    public String backside = "none";
    public String position = "none";

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad1.x || gamepad2.x) {
            alliance_color = kBLUE;
        }
        if (gamepad1.b || gamepad2.b) {
            alliance_color = kRED;
        }
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            current_start_location = kSTART_LOCATION_1;//
        }
        if (gamepad1.dpad_up || gamepad2.dpad_up) {
            current_start_location = kSTART_LOCATION_2;//
        }
        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            current_start_location = kSTART_LOCATION_3;//
        }
        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            current_start_location = kNOT_SET;
        }
        telemetryChoice();
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void telemetryChoice()
    {
        if (alliance_color == kBLUE)
        {
            telemetry.addData("Current Alliance", "Blue");
        }
        else if (alliance_color == kRED)
        {
            telemetry.addData("Current Alliance", "Red");
        }
        else
        {
            telemetry.addData("Current Alliance", "None");
        }
        if (current_start_location == kSTART_LOCATION_1 )
        {
            telemetry.addData("Start Position   ", "Position 1");
        }
        else if (current_start_location == kSTART_LOCATION_2)
        {
            telemetry.addData("Start Position   ", "Position 2");
        }
        else if (current_start_location == kSTART_LOCATION_3)
        {
            telemetry.addData("Start Position   ", "Position 3");
        }
        else
        {
            telemetry.addData("Start Position   ", "None");
        }
    }
}


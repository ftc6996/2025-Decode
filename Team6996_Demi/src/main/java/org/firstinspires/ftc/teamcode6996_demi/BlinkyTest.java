package org.firstinspires.ftc.teamcode6996_demi;
//gamepad1.back    == robot vs field centric
//gamepad1.options == reset heading

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.Blinky;

@TeleOp(name="BlinkyTest", group="TeleOp")
public class BlinkyTest extends OpMode{
    private Blinky blinky;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        blinky = new Blinky();
        blinky.init(hardwareMap, telemetry);
        //blinky.setMatchTimeInSec(120);
    }
    
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("Version", "5");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        blinky.processBlinky();

        blinky.showData();
    }

    @Override
    public void stop() {
        blinky.stop();
    }


    private void mySleep(int milliseconds)
    {
        ElapsedTime timer = new ElapsedTime();
        int x = 0;
        while(timer.milliseconds() < milliseconds)
        {
            x++;
        }
    }
}

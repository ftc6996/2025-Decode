package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Blinky {

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public Blinky()
    {
    }

    public void init(HardwareMap hardwareMap)
    {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void setRedAlliance()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
    }
    public void setBlueAlliance()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE);
    }
    public void setUnknownAlliance()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }
    public void setInRange()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public void process(ElapsedTime current)
    {
        double TELEOP_TIME = 30; //seconds
        double LAST_10_SECONDS = TELEOP_TIME - 10;
        double LAST_20_SECONDS = TELEOP_TIME - 20;
        double LAST_30_SECONDS = TELEOP_TIME - 30;
        if (current.seconds() > LAST_10_SECONDS)
        {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        }
        else if (current.seconds() > LAST_20_SECONDS)
        {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else if (current.seconds() > LAST_30_SECONDS)
        {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
        }
    }
}

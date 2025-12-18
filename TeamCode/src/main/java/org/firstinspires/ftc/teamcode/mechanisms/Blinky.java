package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
}

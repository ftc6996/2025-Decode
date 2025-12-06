package org.firstinspires.ftc.teamcode6996_demi.mechanisms;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Blinky {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    ElapsedTime blinkTimer = new ElapsedTime();

    private int matchTimeInSec = 120;


    final int MINIMUM_MATCH_TIME = 30;
    final int ENTER_SLOW_TIME = 30;
    final int ENTER_MED_TIME = 20;
    final int ENTER_FAST_TIME = 10;

    protected enum LEDPattern
    {
        IDLE,
        FLASHALLIANCESLOW,
        FLASHALLIANCEMED,
        FLASHALLIANCEFAST
    }


    LEDPattern ledPattern = LEDPattern.IDLE;

    public Blinky()
    {
    }

    Telemetry telemetry; // declare

    public void init(HardwareMap hardwareMap, Telemetry telemetry)
    {
        blinkTimer.reset();
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        this.telemetry = telemetry;
    }

    public void showData()
    {
        // show color information value via telemetry
        telemetry.addData("blinkTimer",blinkTimer.seconds());
    }

    public void processBlinky()
    {
        switch(ledPattern)
        {
            case IDLE:
                if (blinkTimer.seconds() > (matchTimeInSec - ENTER_SLOW_TIME))
                {
                    ledPattern = LEDPattern.FLASHALLIANCESLOW;
                }
                break;
            case FLASHALLIANCESLOW:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);

                if (blinkTimer.seconds() > (matchTimeInSec - ENTER_MED_TIME))
                {
                    ledPattern = LEDPattern.FLASHALLIANCEMED;
                }
                break;
            case FLASHALLIANCEMED:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);

                if (blinkTimer.seconds() > (matchTimeInSec - ENTER_FAST_TIME))
                {
                    ledPattern = LEDPattern.FLASHALLIANCEFAST;
                }
                break;
            case FLASHALLIANCEFAST:
                if  (blinkTimer.seconds() > matchTimeInSec)
                {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                } else {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                }

                break;
        }
    }


    public boolean setMatchTimeInSec(int matchTime)
    {
        boolean rtnVal = false;
        if (matchTime >= MINIMUM_MATCH_TIME)
        {
            matchTimeInSec = matchTime;
            rtnVal = true;
        }
        return rtnVal;
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

    public void stop()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDetection {


    public enum colorType {
        COLOR_UNKNOWN,
        COLOR_RED,
        COLOR_PURPLE,
        COLOR_BLUE,
        COLOR_GREEN
    };

    private NormalizedColorSensor colorSensor;

    float gain = 20;

    Telemetry telemetry; // declare

    public void init(HardwareMap hardwareMap, Telemetry telemetry, String sensorName) {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        this.telemetry = telemetry;
    }

    public void showData(String sensorName) {
        // show color information value via telemetry
        telemetry.addData("Gain", gain);

        telemetry.addData("Name", sensorName)
                .addData("TYPE", "%s", getColorTypeString(getColor()));
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);

    }


    public Boolean findColor(colorType detectedColor) {
        boolean foundColor = false;

        if (detectedColor == getColor()) {
            foundColor = true;
        }

        return foundColor;
    }

    public static String getColorTypeString(colorType foundColorType) {
        String strColorType = "COLOR_UNKNOWN";

        if (foundColorType == colorType.COLOR_RED)
        {
            strColorType = "COLOR_RED";
        }
        else if (foundColorType == colorType.COLOR_PURPLE)
        {
            strColorType = "COLOR_PURPLE";
        }
        else if (foundColorType == colorType.COLOR_BLUE)
        {
            strColorType = "COLOR_BLUE";
        }

        else if (foundColorType == colorType.COLOR_GREEN)
        {
            strColorType = "COLOR_GREEN";
        }

        else if (foundColorType == colorType.COLOR_UNKNOWN)
        {
            strColorType = "COLOR_UNKNOWN";
        }
        return strColorType;
    }



    protected colorType getColor() {

        colorType foundColorType = colorType.COLOR_UNKNOWN;
        double distance = -999;
        if (colorSensor instanceof DistanceSensor) {
            distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        }
        NormalizedRGBA color = colorSensor.getNormalizedColors();
        if ((distance > 0) && (distance < 6))
        {
            //color sensor v3

            if ((color.red >= color.blue) && (color.red >= color.green))
            {
                foundColorType = colorType.COLOR_RED;
            }

            else if ((color.blue >= color.red) && (color.blue >= color.green))
            {
                foundColorType = colorType.COLOR_BLUE;
            }

            else if ((color.green >= color.blue) && (color.green >= color.red))
            {
                foundColorType = colorType.COLOR_GREEN;
            }
            else if ((color.blue >= color.green))// && (color.red >= color.green))
            {
                foundColorType = colorType.COLOR_PURPLE;
            }
            else
            {
                foundColorType = colorType.COLOR_UNKNOWN;
            }
        }
        return foundColorType;
    }

}

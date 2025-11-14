/*
Copyright 2025 FIRST Tech Challenge Team 6996

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode6996_demi;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.detectColor;

import org.firstinspires.ftc.teamcode6996_demi.mechanisms.twoWheelDrive;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode6996_demi.mechanisms.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.Range;



public class endgameController {

    private detectColor leftColorSensor;
    private detectColor rightColorSensor;
    private detectColor backColorSensor;

    private boolean leftDetected = false;
    private boolean rightDetected = false;
    public detectColor.colorType ALLIANCE_COLOR = detectColor.colorType.COLOR_BLUE;

    public enum robotStates {
        IDLE_STATE,
        INIT_STATE,
        LOCATE_BASE_ZONE_STATE,
        ENTER_BASE_ZONE_STATE,
        PARK_STATE
    }

    robotStates robotState = robotStates.IDLE_STATE;

    double driveSpeed = 1;
    double leftPower = driveSpeed;
    double rightPower = driveSpeed;

    Telemetry telemetry; // declare

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Define and Initialize color sensors
        leftColorSensor = new detectColor();
        leftColorSensor.init(hardwareMap, telemetry, "sensor_color");

        rightColorSensor = new detectColor();
        rightColorSensor.init(hardwareMap, telemetry, "sensor_color2");

        backColorSensor = new detectColor();
        backColorSensor.init(hardwareMap, telemetry, "sensor_color3");

        robotStates robotState = robotStates.IDLE_STATE;

        this.telemetry = telemetry;

    }



    public void showData() {
        // display color sensor information via telemetry
        leftColorSensor.showData("left Color sensor");
        rightColorSensor.showData("right Color sensor");
        backColorSensor.showData("back Color sensor");
        telemetry.addData("Alliance Color", "%s", detectColor.getColorTypeString( ALLIANCE_COLOR));
    }



    //call it periodically
    //return true when engame control started/in progress
    //return false when endgame control idle
    public boolean processUpdate(MecanumDrive drive, boolean startProcess) {

        boolean returnValue = true;

        switch (robotState) {
            case IDLE_STATE:
                if (startProcess) {
                    robotState = robotStates.INIT_STATE;
                } else {
                    returnValue = false;
                }
                break;

            case INIT_STATE:
                drive.setPower(leftPower, rightPower, leftPower, rightPower);
                robotState = robotStates.LOCATE_BASE_ZONE_STATE;
                leftDetected = false;
                rightDetected = false;
                break;

            case LOCATE_BASE_ZONE_STATE:
                if (leftColorSensor.findColor(ALLIANCE_COLOR)) {
                    leftDetected = true;
                    leftPower = 0;
                    drive.setPower(leftPower, rightPower, leftPower, rightPower);
                }

                if (rightColorSensor.findColor(ALLIANCE_COLOR)) {
                    rightDetected = true;
                    rightPower = 0;
                    drive.setPower(leftPower, rightPower, leftPower, rightPower);
                }

                // When both sensors have detected the color
                if (leftDetected && rightDetected) {
                    robotState = robotStates.ENTER_BASE_ZONE_STATE;
                }
                break;

            case ENTER_BASE_ZONE_STATE:
                leftPower = driveSpeed;
                rightPower = driveSpeed;
                drive.setPower(leftPower, rightPower, leftPower, rightPower);
                robotState = robotStates.PARK_STATE;
                break;

            case PARK_STATE:
                if (backColorSensor.findColor(ALLIANCE_COLOR)) {
                    // Stop all motion
                    drive.stop();
                    robotState = robotStates.IDLE_STATE;
                }
                break;
        }
        return returnValue;

    }
    public boolean processUpdate(twoWheelDrive drive, boolean startProcess) {

        boolean returnValue = true;

        switch (robotState) {
            case IDLE_STATE:
                if (startProcess) {
                    robotState = robotStates.INIT_STATE;
                } else {
                    returnValue = false;
                }
                break;

            case INIT_STATE:
                drive.setPower(leftPower, rightPower, leftPower, rightPower);
                robotState = robotStates.LOCATE_BASE_ZONE_STATE;
                leftDetected = false;
                rightDetected = false;
                break;

            case LOCATE_BASE_ZONE_STATE:
                if (leftColorSensor.findColor(ALLIANCE_COLOR)) {
                    leftDetected = true;
                    leftPower = 0;
                    drive.setPower(leftPower, rightPower, leftPower, rightPower);
                }

                if (rightColorSensor.findColor(ALLIANCE_COLOR)) {
                    rightDetected = true;
                    rightPower = 0;
                    drive.setPower(leftPower, rightPower, leftPower, rightPower);
                }

                // When both sensors have detected the color
                if (leftDetected && rightDetected) {
                    robotState = robotStates.ENTER_BASE_ZONE_STATE;
                }
                break;

            case ENTER_BASE_ZONE_STATE:
                leftPower = driveSpeed;
                rightPower = driveSpeed;
                drive.setPower(leftPower, rightPower, leftPower, rightPower);
                robotState = robotStates.PARK_STATE;
                break;

            case PARK_STATE:
                if (backColorSensor.findColor(ALLIANCE_COLOR)) {
                    // Stop all motion
                    drive.stop();
                    robotState = robotStates.IDLE_STATE;
                }
                break;
        }
        return returnValue;

    }

    public void switchAllianceColor() {
        if (ALLIANCE_COLOR == detectColor.colorType.COLOR_BLUE) {
            ALLIANCE_COLOR = detectColor.colorType.COLOR_RED;
        } else {
            ALLIANCE_COLOR = detectColor.colorType.COLOR_BLUE;
        }
    }

    public void adjustDriveSpeed(double adjustDriveSpeed) {
        driveSpeed = Range.clip(driveSpeed+adjustDriveSpeed,0,1);
        leftPower = driveSpeed;
        rightPower = driveSpeed;
    }
}

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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.ColorDetection;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;



public class EndgameController {

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private boolean backSensorInstall = false;
    private ColorDetection leftColorSensor;
    private ColorDetection rightColorSensor;
    private ColorDetection backColorSensor;

    private ElapsedTime enterParkTimer = new ElapsedTime();

    private boolean leftDetected = false;
    private boolean rightDetected = false;
    public ColorDetection.colorType ALLIANCE_COLOR = ColorDetection.colorType.COLOR_BLUE;


    public enum endgameCommands {
        START,
        CANCEL,
    }

    public enum robotStates {
        IDLE_STATE,
        INIT_STATE,
        LOCATE_BASE_ZONE_STATE,
        ENTER_BASE_ZONE_STATE,
        PARK_STATE,
        CANCEL_STATE
    }

    robotStates robotState = robotStates.IDLE_STATE;

    double driveSpeed = 1;
    double leftFrontPower = -driveSpeed;
    double rightFrontPower = -driveSpeed;
    double leftBackPower = -driveSpeed;
    double rightBackPower = -driveSpeed;

    Telemetry telemetry; // declare

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        // Define and Initialize color sensors
        leftColorSensor = new ColorDetection();
        leftColorSensor.init(hardwareMap, telemetry, "sensor_color");

        rightColorSensor = new ColorDetection();
        rightColorSensor.init(hardwareMap, telemetry, "sensor_color2");

        if (backSensorInstall) {
            backColorSensor = new ColorDetection();
            backColorSensor.init(hardwareMap, telemetry, "sensor_color3");
        }

        robotStates robotState = robotStates.IDLE_STATE;

        this.telemetry = telemetry;

    }



    public void showData() {
        // display color sensor information via telemetry
        leftColorSensor.showData("left Color sensor");
        rightColorSensor.showData("right Color sensor");
        if (backSensorInstall) {
            backColorSensor.showData("back Color sensor");
        }
        telemetry.addData("Alliance Color", "%s", ColorDetection.getColorTypeString( ALLIANCE_COLOR));
    }



    //call it periodically
    //return true when endgame control started/in progress
    //return false when endgame control idle
    public boolean processUpdate(MecanumDrive drive, endgameCommands command) {

        boolean returnValue = true;

        if (command == endgameCommands.CANCEL) {
            robotState = robotStates.CANCEL_STATE;
        }

        switch (robotState) {
            case IDLE_STATE:
                if (command == endgameCommands.START) {
                    robotState = robotStates.INIT_STATE;
                } else {
                    returnValue = false;
                }
                break;

            case INIT_STATE:
                drive.setPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                robotState = robotStates.LOCATE_BASE_ZONE_STATE;
                leftDetected = false;
                rightDetected = false;
                break;

            case LOCATE_BASE_ZONE_STATE:
                if (leftColorSensor.findColor(ALLIANCE_COLOR)) {
                    leftDetected = true;
                    leftBackPower = 0;
                    leftFrontPower = 0;
                    drive.setPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                }

                if (rightColorSensor.findColor(ALLIANCE_COLOR)) {
                    rightDetected = true;
                    rightFrontPower = 0;
                    rightBackPower = 0;
                    drive.setPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                }

                // When both sensors have detected the color
                if ( leftDetected  && rightDetected) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                    robotState = robotStates.ENTER_BASE_ZONE_STATE;
                }
                break;

            case ENTER_BASE_ZONE_STATE:
                leftBackPower = driveSpeed;
                leftFrontPower = driveSpeed;
                rightFrontPower = driveSpeed;
                rightBackPower = driveSpeed;
                drive.setPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                robotState = robotStates.PARK_STATE;
                if (!backSensorInstall) {
                    enterParkTimer.reset();
                }
                break;

            case PARK_STATE:
                if (backSensorInstall) {
                    if (backColorSensor.findColor(ALLIANCE_COLOR)) {
                        // Stop all motion
                        drive.stop();
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
                        robotState = robotStates.IDLE_STATE;
                    } else {
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                    }

                } else {
                    if (enterParkTimer.seconds() > 3) {
                        // Stop all motion
                        drive.stop();
                        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
                        robotState = robotStates.IDLE_STATE;
                    }
                }
                break;

            case CANCEL_STATE:
                drive.stop();
                robotState = robotStates.IDLE_STATE;
                returnValue = false;
                break;
        }
        return returnValue;

    }
//    public boolean processUpdate(TwoWheelDrive drive, boolean startProcess) {
//
//        boolean returnValue = true;
//
//        switch (robotState) {
//            case IDLE_STATE:
//                if (startProcess) {
//                    robotState = robotStates.INIT_STATE;
//                } else {
//                    returnValue = false;
//                }
//                break;
//
//            case INIT_STATE:
//                drive.setPower(leftFrontPower, rightFrontPower);
//                robotState = robotStates.LOCATE_BASE_ZONE_STATE;
//                leftDetected = false;
//                rightDetected = false;
//                break;
//
//            case LOCATE_BASE_ZONE_STATE:
//                if (leftColorSensor.findColor(ALLIANCE_COLOR)) {
//                    leftDetected = true;
//                    leftFrontPower = 0;
//                    drive.setPower(leftFrontPower, rightFrontPower);
//                }
//
//                if (rightColorSensor.findColor(ALLIANCE_COLOR)) {
//                    rightDetected = true;
//                    rightFrontPower = 0;
//                    drive.setPower(leftFrontPower, rightFrontPower);
//                }
//
//                // When both sensors have detected the color
//                if (leftDetected && rightDetected) {
//                    robotState = robotStates.ENTER_BASE_ZONE_STATE;
//                }
//                break;
//
//            case ENTER_BASE_ZONE_STATE:
//                leftFrontPower = driveSpeed;
//                rightFrontPower = driveSpeed;
//                drive.setPower(leftFrontPower, rightFrontPower);
//                robotState = robotStates.PARK_STATE;
//                break;
//
//            case PARK_STATE:
//                if (backColorSensor.findColor(ALLIANCE_COLOR)) {
//                    // Stop all motion
//                    drive.stop();
//                    robotState = robotStates.IDLE_STATE;
//                }
//                break;
//        }
//        return returnValue;
//
//    }

    public void switchAllianceColor() {
        if (ALLIANCE_COLOR == ColorDetection.colorType.COLOR_BLUE) {
            ALLIANCE_COLOR = ColorDetection.colorType.COLOR_RED;
        } else {
            ALLIANCE_COLOR = ColorDetection.colorType.COLOR_BLUE;
        }
    }

    public void adjustDriveSpeed(double adjustDriveSpeed) {
        driveSpeed = Range.clip(driveSpeed+adjustDriveSpeed,0,1);
        leftBackPower = driveSpeed;
        leftFrontPower = driveSpeed;
        rightFrontPower = driveSpeed;
        rightBackPower = driveSpeed;
    }
}

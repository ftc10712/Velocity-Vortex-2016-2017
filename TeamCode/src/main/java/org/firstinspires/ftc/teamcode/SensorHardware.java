package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;


/**
 * Created by Austin Ford and Tristan Sorenson on 12/18/2016.
 *
 */

class SensorHardware extends ServoMotorHardware {

    ColorSensor colorSensor1 = null;
    TouchSensor touchSensor = null;

    //Sample Code Being Worked On



    //SensorHardware Constructor
    SensorHardware() {

    }

    void initializeSensors(HardwareMap hwMap) {

        //Initialize and Map Color Sensor
        try {
            colorSensor1 = hwMap.colorSensor.get("mr_color_sensor1");
        } catch (Exception p_exception) {
            colorSensor1 = null;
        }

        try {
            touchSensor = hwMap.touchSensor.get("mr_touch_sensor");
        } catch (Exception p_exception) {
            touchSensor = null;
        }

        //Sample Code Being Worked On
    }

    //-----------------------------Sensor Methods begin here--------------------------------------//

    //Report whether or not MR Color Sensor 1 was able to be initialized
    public String getColorSensor1Status() {
        String colorSensorStatus;
        if (colorSensor1 != null) {
            colorSensorStatus = "Initialized";
        } else {
            colorSensorStatus = "Failed to Initialize";
        }
        return colorSensorStatus;
    }

    //Get the Color MR Color Sensor 1 is currently seeing
    String getColorSensor1ColorDetected() {
        String colorDetected;
        if (colorSensor1.blue() >= 1) {
            colorDetected = "Blue";
        } else if (colorSensor1.red() >= 1) {
            colorDetected = "Red";
        } else {
            colorDetected = "No Color Detected";
        }
        return colorDetected;
    }

    //Report whether or not Touch Sensor was able to be initialized
    public String getTouchSensorStatus() {
        String touchSensorStatus;
        if (touchSensor != null) {
            touchSensorStatus = "Initialized";
        } else {
            touchSensorStatus = "Failed to Initialize";
        }
        return touchSensorStatus;
    }

    //Determine if Touch Sensor Button is pressed and return current state.
    String isTouchSensorPressed (){
        String touchSensorState;
        if(touchSensor.isPressed()){
            touchSensorState = "Pressed";
        } else {
            touchSensorState = "Not Pressed";
        }
        return touchSensorState;

    }
}






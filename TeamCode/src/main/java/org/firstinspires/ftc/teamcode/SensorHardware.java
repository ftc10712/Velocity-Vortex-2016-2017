package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Austin Ford and Tristan Sorenson on 12/18/2016.
 * <p>
 * Test of our commit repo..
 */

class SensorHardware extends ServoMotorHardware {

    ColorSensor colorSensor1 = null;
    ColorSensor colorSensor2 = null;
    OpticalDistanceSensor opticalDistanceSensor1 = null;
    OpticalDistanceSensor opticalDistanceSensor2 = null;
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
            colorSensor2 = hwMap.colorSensor.get("mr_color_sensor2");
            colorSensor2.setI2cAddress(I2cAddr.create8bit(0x4c));  //All MR Color Sensors Defalut to 0x3c.  We change it here to 0x4c so it doesn't conflict with Color Sensor 1
            colorSensor2.enableLed(false);
        } catch (Exception p_exception) {
            colorSensor2 = null;
        }

        try {
            opticalDistanceSensor1 = hwMap.opticalDistanceSensor.get("mr_optical_distance_sensor1");
        } catch (Exception p_exception) {
            opticalDistanceSensor1 = null;
        }

        try {
            opticalDistanceSensor2 = hwMap.opticalDistanceSensor.get("mr_optical_distance_sensor2");
        } catch (Exception p_exception) {
            opticalDistanceSensor2 = null;
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
        String colorSensor1Status;
        if (colorSensor1 != null) {
            colorSensor1Status = "Initialized";
        } else {
            colorSensor1Status = "Failed to Initialize";
        }
        return colorSensor1Status;

    }

    //Get the Color MR Color Sensor 1 is currently seeing
    String getColorSensor1ColorDetected() {
        String color1Detected;
        if (colorSensor1.blue() >= 1) {
            color1Detected = "Blue";
        } else if (colorSensor1.red() >= 1) {
            color1Detected = "Red";
        } else {
            color1Detected = "No Color Detected";
        }
        return color1Detected;

    }

    //Report whether or not MR Color Sensor 1 was able to be initialized
    public String getColorSensor2Status() {
        String colorSensor2Status;
        if (colorSensor1 != null) {
            colorSensor2Status = "Initialized";
        } else {
            colorSensor2Status = "Failed to Initialize";
        }
        return colorSensor2Status;
    }

    //Get the Color MR Color Sensor 2 is currently seeing
    String getColorSensor2ColorDetected() {
        String color2Detected;
        if (colorSensor2.blue() >= 1) {
            color2Detected = "Blue";
        } else if (colorSensor2.red() >= 1) {
            color2Detected = "Red";
        } else {
            color2Detected = "No Color Detected";
        }
        return color2Detected;
    }

    //Report whether or not MR Optical Distance Sensor 1 was able to be initialized
    public String getOpticalDistanceSensor1Status() {
        String opticalDistanceSensor1Status;
        if (opticalDistanceSensor1 != null) {
            opticalDistanceSensor1Status = "Initialized";
        } else {
            opticalDistanceSensor1Status = "Failed to Initialize";
        }
        return opticalDistanceSensor1Status;
    }

    //Get the raw light value that Optical Distance Sensor 1 is currently seeing
    double getOpticalDistanceSensor1RawLightDetected() {
        return opticalDistanceSensor1.getRawLightDetected();
    }

    //Get the normal light value that Optical Distance Sensor 1 is currently seeing
    double getOpticalDistanceSensor1NormalLightDettected() {
        return opticalDistanceSensor1.getLightDetected();
    }

    //Turn on Optical Distance Sensor 1 LED Example
    void turnOnOpiticalDistanceSensor1LED() {
        opticalDistanceSensor1.enableLed(true);
    }

    //Report whether or not MR Optical Distance Sensor 2 was able to be initialized
    public String getOpticalDistanceSensor2Status() {
        String opticalDistanceSensor2Status;
        if (opticalDistanceSensor2 != null) {
            opticalDistanceSensor2Status = "Initialized";
        } else {
            opticalDistanceSensor2Status = "Failed to Initialize";
        }
        return opticalDistanceSensor2Status;
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
    String isTouchSensorPressed() {
        String touchSensorState;
        if (touchSensor.isPressed()) {
            touchSensorState = "Pressed";
        } else {
            touchSensorState = "Not Pressed";
        }
        return touchSensorState;

    }
}






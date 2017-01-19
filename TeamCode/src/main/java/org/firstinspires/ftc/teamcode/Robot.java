package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Program:     Robot
 *
 * @author Created by Austin Ford and Tristan Sorenson
 *
 * @since December 18, 2016
 *
 * Compiler:    JDK 1.8.0_112
 *
 * Platform:    Android - KitKat 4.4.4
 *
 * Hardware:    ZTE Speed - Model 9130
 *
 * Description: This is the main class that is called to initialize all robot hardware and senors
 *              and setup all methods to drive hardware and sensors.  It is intended to be called
 *              as an instantiated class from TeleOp or Autonomous modes.
 *
 *
 */

class Robot extends JoystickHardware {

    //Robot Class Constructor
    public Robot() {

    }

    //Member Variables
    //ElapsedTime period = new ElapsedTime();

    /**
     * Method:          initializeRobot
     * Purpose:         Calls child classes to initialize robot hardware and sensors.
     * @param hwMap     Copy of Hardware Map to send to child classes to initialize roboto hardware
     *                  and sensors.
     * @return None
     */

    void initializeRobot(HardwareMap hwMap) {
        initializeDCMotors(hwMap);
        initializeServoMotors(hwMap);
        initializeSensors(hwMap);
    }

}

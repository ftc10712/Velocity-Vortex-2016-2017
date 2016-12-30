package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Austin Ford and Tristan Sorenson 12/18/2016.
 *
 */

class Robot extends SensorHardware {

    //Robot Class Constructor
    public Robot() {

    }

    //Member Variables

    ElapsedTime period = new ElapsedTime();

    /**
     *
     * @param hwMap - Copy of Hardware Map to Send to Classes to Initialize Robot
     *
     */
    void initializeRobot(HardwareMap hwMap) {
        //initializeDCMotors(hwMap);
        initializeServoMotors(hwMap);
        initializeSensors(hwMap);
    }

}

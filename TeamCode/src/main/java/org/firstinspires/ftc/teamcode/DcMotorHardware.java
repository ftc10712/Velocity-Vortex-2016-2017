package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Austin Ford & Tristan Sorenson on 12/15/2016.
 *
 */


class DcMotorHardware {

    private DcMotor rightFrontMotor;
    private DcMotor rightRearMotor;
    private DcMotor leftFrontMotor;
    private DcMotor leftRearMotor;

    //DCMotorHardware Class Constructor

    DcMotorHardware() {

    }

    /**
     * Called by Robot Class and passed the Hardware Map reference to initialize DC Motors
     *
     * @param hwMap - This is a reference to the hardeware map
     */
    void initializeDCMotors(HardwareMap hwMap) {

        /**
         * Try to map the drive DC motors to the mapped devices on the android phone
         * if they fail, create an entry in the logcat file on the android phone
         * to be viewed later.
         */
        try {
            rightFrontMotor = hwMap.dcMotor.get("right_front_motor");
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightFrontMotor = null;

        }

        try {
            rightRearMotor = hwMap.dcMotor.get("right_rear_motor");
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            rightRearMotor = null;
        }

        try {
            leftFrontMotor = hwMap.dcMotor.get("left_front_motor");
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftFrontMotor = null;
        }

        try {
            leftRearMotor = hwMap.dcMotor.get("left_rear_motor");
        } catch (Exception errorMessage) {
            DbgLog.msg(errorMessage.getLocalizedMessage());
            leftRearMotor = null;
        }
    }


}

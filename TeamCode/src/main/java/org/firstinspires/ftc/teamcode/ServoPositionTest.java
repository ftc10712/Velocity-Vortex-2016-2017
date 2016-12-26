package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Darrell on 12/17/2016.
 */

public class ServoPositionTest {

    protected Servo rightForkDeployServo;

    static final double DEPLOY_FORKS    =  1.4;     // Maximum rotational position
    static final double RETRACT_FORKS    =  -0.4;     // Minimum rotational position

    HardwareMap hwMap = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public ServoPositionTest() {

    }

    public void init(HardwareMap ahwMap) {


        // Save reference to Hardware map
        hwMap = ahwMap;

        try {
            rightForkDeployServo = hwMap.servo.get("right_fork_deploy_servo");
        } catch (Exception p_exeception) {
            rightForkDeployServo = null;
        }
    }

    public void deployForks (){
        //rightForkDeployServo.setDirection(Servo.Direction.FORWARD);
        rightForkDeployServo.setPosition(DEPLOY_FORKS);
    }

    public void retractForks(){
        //rightForkDeployServo.setDirection(Servo.Direction.REVERSE);
        rightForkDeployServo.setPosition(RETRACT_FORKS);
    }

}

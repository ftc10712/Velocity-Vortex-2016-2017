package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class SheldonMotorHardware {


    /*----------- Start DC Motor Variable Definitions (8 total DC Motors) ------------------*/

    /* Define variables for DC drive motors on Sheldon and initialize them to null */
    public DcMotor  leftRearMotor       = null;
    public DcMotor  rightRearMotor      = null;
    public DcMotor  leftFrontMotor      = null;
    public DcMotor  rightFrontMotor     = null;

    /* Define variable for DC motor that will lift the mast on Sheldon and initialize it to null */
/*    public DcMotor liftMastMotor        = null;*/

    /* Define variables for DC motors that will throw the particle balls on Sheldon and initialize it to null */
/*    public DcMotor leftParticleMotor    = null;
    public DcMotor rightParticleMotor   = null;*/

    /* Define variable for DC motor that will turn the conveyor to direct particle balls to particle shooter on Sheldon and initialize it to null */
    public DcMotor conveyorMotor        = null;

    /*---------------------- End DC Motor Variable Definitions -----------------------------*/

    /*-------------- Start Servo Motor Variable Definitions (9 Total Servos) ---------------*/

    /* Define variables for Fork Deployment Servos and initialize them to null */
/*    public Servo    leftForkDeployServo      = null;
    public Servo    rightForkDeployServo     = null;*/


    /* Define variables for Fork Gripper Servos and initialize them to null */
    public CRServo    leftForkGripperServo    = null;
    public CRServo rightForkGripperServo   = null;

    /* Define variables for Mast Horizontal Movement Servos and initialize them to null */
/*    public CRServo    leftMastHorizontalMovementServo     = null;
    public CRServo    rightMastHorizontalMovementServo    = null;*/

    /* Define variables for Servos that will deploy Beacon Pushing Mechanisms and initialize them to null */
/*    public Servo    leftBeaconServo     = null;
    public Servo    rightBeaconServo    = null;*/

    /* Define variable for Turret Direction Servo and initialize them to null */
/*    public CRServo    turretServo     = null;*/

    /*------------------------- End Servo Motor Variable Definitions -----------------------*/




    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SheldonMotorHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
/*
        // Define and Initialize Motors
        leftRearMotor   = hwMap.dcMotor.get("left_rear_motor");
        rightRearMotor  = hwMap.dcMotor.get("right_rear_motor");
        leftFrontMotor   = hwMap.dcMotor.get("left_front_motor");
        rightFrontMotor  = hwMap.dcMotor.get("right_front_motor");

        //armMotor    = hwMap.dcMotor.get("left_arm");
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        //armMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftForkDeployServo = hwMap.servo.get("left_fork_deploy_servo");
        rightForkDeployServo = hwMap.servo.get("righ_fork_deploy_servo");
*/
        leftForkGripperServo = hwMap.crservo.get("left_fork_gripper_servo");
        rightForkGripperServo = hwMap.crservo.get("right_fork_gripper_servo");
/*        leftMastHorizontalMovementServo = hwMap.crservo.get("left_mast_horizontal_servo");
        rightMastHorizontalMovementServo = hwMap.crservo.get("right_mast_horizontal_servo");
        leftBeaconServo = hwMap.servo.get("left_beacon_servo");
        rightBeaconServo = hwMap.servo.get("right_beacon_servo");
        turretServo = hwMap.crservo.get("turret_servo");
*/
        //rightClaw = hwMap.servo.get("right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}


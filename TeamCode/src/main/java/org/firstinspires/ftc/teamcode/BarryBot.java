package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.libraries.DrivingLibrary;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left front motor:         "leftFront"
 * Motor channel:  Right front motor:        "rightFront"
 * Motor channel:  Left rear motor:          "leftRear"
 * Motor channel:  Right rear motor:         "rightRear"
 * Motor channel:  Arm motor;                "grabArm"
 * Servo channel:  Hand servo;               "grabHand"
 * Servo channel:  Servo move base;          "dragArm"
 */

public class BarryBot {
    /* Public OpMode members. */
    public DcMotor  leftFront   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  leftRear    = null;
    public DcMotor  rightRear   = null;
    public DcMotor  grabArm     = null;
    public Servo    grabHand    = null;
    public Servo    dragArm     = null;

    public DrivingLibrary drivingLibrary = null;
    public int drivingMode = 0;

    public static final double MID_SERVO       =  0.5 ;
    /*public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public BarryBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, OpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftRear  = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        grabArm    = hwMap.get(DcMotor.class, "grabArm");
        //leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        grabArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grabArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        grabHand  = hwMap.get(Servo.class, "grabHand");
        dragArm = hwMap.get(Servo.class, "dragArm");
        grabHand.setPosition(MID_SERVO);
        dragArm.setPosition(MID_SERVO);

        drivingLibrary = new DrivingLibrary(opMode);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        //adjusting strafe bias because FR wheel moves slower
        drivingLibrary.setStrafeBias(1, 1.05);
    }
}

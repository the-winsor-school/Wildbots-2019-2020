package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Landing Only", group = "Finished")
public class AutonLand extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    // latch arm
    DcMotor latchArm;

    /*// intake arm
    CRServo intakeSpinServo;
    Servo intakeFlipServo;
    Servo intakeExtendArm;
    DcMotor intakeRotateArm;
    Servo drawerStopServo;
    */

    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        /*
        // intake cr servo: rev hub 1 servo port 0
        intakeSpinServo = hardwareMap.get(CRServo.class, "intakeSpinServo");

        // intake flip servo: rev hub 1 servo port 1
        intakeFlipServo = hardwareMap.get(Servo.class, "intakeFlipServo");

        // intake arm extending winch servo: rev hub 1 servo port 2
        intakeExtendArm = hardwareMap.get(Servo.class, "intakeExtendArm");

        // intake arm rotational dc motor: rev hub 1 motor port 1
        intakeRotateArm = hardwareMap.get(DcMotor.class, "intakeRotateArm");
        */

        // latch motor: rev hub 1 motor port 0
        latchArm = hardwareMap.get(DcMotor.class, "latchArm");

        // drawerStopServo = hardwareMap.get(Servo.class, "drawerStopServo");

        latchArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // intakeRotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean ranOnce = false;

        // drawerStopServo.setPosition(0.9);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){
                //land
                latchArm.setPower(1);
                sleep(28000);

                sleep(100);

                drivingLibrary.turn(.5f,.5f);
                sleep(500);
                drivingLibrary.brakeStop();

                //sleep(2000);

                /* //reset arm
                latchArm.setPower(1);
                sleep(1000);

                sleep(2000);
                 */

                // drawerStopServo.setPosition(0.39);


            }
            ranOnce = true;
        }
    }

}

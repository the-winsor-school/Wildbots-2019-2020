package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "TeleOp Mode", group = "Finished")
public class TeleOpMode extends LinearOpMode {

    // drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    // latch arm
    DcMotor latchArm;

    DcMotor testIntake;

    /* // intake arm
    CRServo intakeSpinServo;
    Servo intakeFlipServo;
    Servo intakeExtendArm;
    DcMotor intakeRotateArm;
    Servo drawerStopServo;
    */

    public void runOpMode() throws InterruptedException {
        // set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        /* // intake cr servo: rev hub 1 servo port 0
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

        testIntake = hardwareMap.get(DcMotor.class, "testIntake");

        //turn on intake forwards? (idk which direction is which)
        if (gamepad2.a) {
            testIntake.setPower(-.5);
        }
        //turn off intake
        if (gamepad2.b){
            testIntake.setPower(0);
        }

        /* // rev hub 1 port 3
        drawerStopServo = hardwareMap.get(Servo.class, "drawerStopServo");

        latchArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        */

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* boolean holdA = false;
        boolean holdB = false;
        */



        waitForStart();

        // drawerStopServo.setPosition(0.9);

        while (opModeIsActive()) {
            // game pad 1: driving and latch

            // switch driving modes manually
            if (gamepad1.b) {
                drivingMode++;
                drivingMode %= DrivingMode.values().length;
                drivingLibrary.setMode(drivingMode);
            }

            // drive straight
            drivingLibrary.driveStraight(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            drivingLibrary.turn(gamepad1.right_stick_x, -gamepad1.right_stick_y);

            /* // drawer stop off
            if (gamepad1.y) {
                drawerStopServo.setPosition(.9);
            } else if (gamepad1.x) {
                drawerStopServo.setPosition(.39);
            }
            */

            // control latch arm
            if (gamepad2.dpad_up) {
                latchArm.setPower(-1);
            } else if (gamepad2.dpad_down) {
                latchArm.setPower(1);
            } else if (gamepad2.dpad_right) {
                latchArm.setPower(-.5);
            } else if (gamepad2.dpad_left) {
                latchArm.setPower(.5);
            } else {
                latchArm.setPower(0);
            }

            /* // gamepad 2: intake and intake arm

            // extend intake arm (winch servo)
            if (gamepad2.dpad_right) {
                intakeExtendArm.setPosition(.7);
            } else if (gamepad2.dpad_left) {
                intakeExtendArm.setPosition(.2);
            }

            // rotate intake arm (motor)
            if (gamepad2.dpad_up) {
                intakeRotateArm.setPower(.5);
            } else if (gamepad2.dpad_down) {
                intakeRotateArm.setPower(-.5);
            } else {
                intakeRotateArm.setPower(0);
            }

            // spinning intake servo
            if (gamepad2.a) {
                if (holdA) {
                    intakeSpinServo.setPower(0);
                    holdA = false;
                    holdB = false;
                } else {
                    intakeSpinServo.setPower(.9);
                    holdA = true;
                    holdB = false;
                }

            } else if (gamepad2.b) {
                if (holdB) {
                    intakeSpinServo.setPower(0);
                    holdB = false;
                    holdA = false;
                } else {
                    intakeSpinServo.setPower(-.9);
                    holdB = true;
                    holdA = false;
                }
            }

            // flip servo - play with these values
            if (gamepad2.y) {
                intakeFlipServo.setPosition(.1);
            } else if (gamepad2.x) {
                intakeFlipServo.setPosition(.3);
            }
            */

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update();
        }
    }
}
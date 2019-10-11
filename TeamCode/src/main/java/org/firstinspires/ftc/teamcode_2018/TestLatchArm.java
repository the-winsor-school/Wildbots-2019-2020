package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "Test Latch Mech", group = "Test")
public class TestLatchArm extends LinearOpMode {
    //DrivingLibrary drivingLibrary;
    DcMotor latchArm;
    int drivingMode;

    public void runOpMode() throws InterruptedException {
        /*drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);*/

        // initialize the arm
        latchArm = hardwareMap.get(DcMotor.class, "latchArm");
        latchArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //latchArm.setPower(.75);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
           /* if (gamepad1.b) {
                drivingMode++;
                drivingMode %= DrivingMode.values().length;
                drivingLibrary.setMode(drivingMode);
            }*/

            if (gamepad1.dpad_up) {
                latchArm.setPower(-1);
            } else if (gamepad1.dpad_down) {
                latchArm.setPower(.5);
            } else {
                latchArm.setPower(0);
            }

           /* drivingLibrary.driveStraight(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            drivingLibrary.turn(gamepad1.right_stick_x, -gamepad1.right_stick_y);*/

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", latchArm.getZeroPowerBehavior());

            telemetry.update();
        }
    }
}

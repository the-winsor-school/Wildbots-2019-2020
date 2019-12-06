package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name  = "Test Strafing", group = "Testing")
public class TestStrafing extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    public void runOpMode() throws InterruptedException {
        System.out.println("i'm here");
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            drivingLibrary.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", drivingLibrary.getMode());

            telemetry.update();
        }
    }
}
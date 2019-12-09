package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Red Autonomous")

public class RedAuton extends LinearOpMode {
    DrivingLibrary drivingLibrary;
    int drivingMode;
    DcMotor baseArm;
    DcMotor grabArm;

    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        baseArm = hardwareMap.get(DcMotor.class, "dragArm");
        grabArm = hardwareMap.get(DcMotor.class, "grabArm");

        boolean ranOnce = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){
                drivingLibrary.drive(0f, .5f, 0f);
                sleep(850);
                drivingLibrary.brakeStop();
                sleep(2000);
                drivingLibrary.drive(-.5f, 0f, 0f);
                sleep(1250);
                drivingLibrary.brakeStop();
                sleep(2000);
                baseArm.setPower(0.5);
                sleep(2000);
                drivingLibrary.drive(0f,-.5f, 0f);
                sleep(5500);
                drivingLibrary.brakeStop();
                baseArm.setPower(0);
                sleep(2000);
                baseArm.setPower(-0.5);
                sleep(500);
                baseArm.setPower(0);
                sleep(2000);
                grabArm.setPower(1);
                sleep(750);
                grabArm.setPower(0);
                drivingLibrary.drive(.5f, 0, 0);
                sleep(2850);
                drivingLibrary.brakeStop();
            }
            ranOnce = true;
        }
    }
}
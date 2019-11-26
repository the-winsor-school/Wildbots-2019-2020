package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name  = "TeleOp Mode", group = "Finished")
public class TeleOpMode extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor grabArm;
    DcMotor baseArm;
    Servo grabHand;



    public void runOpMode() throws InterruptedException {
        //set up our driving library
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        grabHand = hardwareMap.get(Servo.class, "grabHand");
        baseArm = hardwareMap.get(DcMotor.class, "baseArm");
        grabArm = hardwareMap.get(DcMotor.class, "grabArm");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double servoPos = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                drivingMode++;
                drivingMode %= DrivingMode.values().length;
                drivingLibrary.setMode(drivingMode);
            }

            //strafe
            drivingLibrary.strafe(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            drivingLibrary.turn(gamepad1.right_stick_x, -gamepad1.right_stick_y);

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", drivingLibrary.getMode());



            //grab arm - the one that holds the 🅱️loccs



            if (gamepad2.x) {
                grabArm.setPower(0.7);
            }
            else if (gamepad2.y) {
                grabArm.setPower(-0.7);
            }
            else {
                grabArm.setPower(0);
            }

            // base arm - the one that moves the 🅱️ase


            if (gamepad2.a) {
                baseArm.setPower(0.7);

            }
            if (gamepad2.b) {
                baseArm.setPower(-0.7);
            }

            else{
                baseArm.setPower(0);

            }

            if (gamepad2.dpad_left) {
                servoPos += .001;
                grabHand.setPosition(servoPos);
            }

            else if (gamepad2.dpad_right && servoPos >= .05) {
                servoPos -= .001;
                grabHand.setPosition(servoPos);

            }

            telemetry.addData("Servo Position: ", servoPos);
            telemetry.addData( "Status:", "Running");

            telemetry.update();





        }
    }
}
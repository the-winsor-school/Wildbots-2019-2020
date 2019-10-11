package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;

@TeleOp(name = "Test Current Intake", group = "Test")
public class TestNewIntake extends LinearOpMode {
    CRServo spinAxle;
    Servo flipBox;
    DcMotor flipArm;
    DcMotor extendArm;
    double servoPos;

    public void runOpMode() throws InterruptedException {
        spinAxle = hardwareMap.get(CRServo.class, "spinAxle");
        flipBox = hardwareMap.get(Servo.class, "flipBox");
        flipArm = hardwareMap.get(DcMotor.class, "flipArm");
        extendArm = hardwareMap.get(DcMotor.class, "extendArm");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                spinAxle.setPower(-.5);
            }
            else if (gamepad1.b) {
                spinAxle.setPower(.5);
            }
            else {
                spinAxle.setPower(0);
            }
            if (gamepad1.x) {
                servoPos += .0001;
                flipBox.setPosition(servoPos);
            }
            else if (gamepad1.y) {
                servoPos -= .0001;
                flipBox.setPosition(servoPos);
            }
            //out
            if (gamepad1.dpad_right) {
                extendArm.setPower(.5);
            }
            //in
            else if (gamepad1.dpad_left) {
                extendArm.setPower(-.5);
            }
            else {
                extendArm.setPower(0);
            }
            if (gamepad1.dpad_up) {
                flipArm.setPower(.5);
            }
            else if (gamepad1.dpad_down) {
                flipArm.setPower(-.5);
            }
            else {
                flipArm.setPower(0);
            }

        }
    }
}
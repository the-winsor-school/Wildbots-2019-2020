package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Test Winch Servo Positions for Box", group = "Test")
public class TestWinchServo extends LinearOpMode {
    Servo flipBox;
    double servoPos = .091;

    public void runOpMode() throws InterruptedException {
        flipBox = hardwareMap.get(Servo.class, "flipBox");

        waitForStart();

        while (opModeIsActive()) {
            flipBox.setPosition(servoPos);
            if (gamepad1.a) {
                servoPos += .0001;
            }
            if (gamepad1.b) {
                servoPos -= .0001;
            }

            telemetry.addData("Servo Position", flipBox.getPosition());
            telemetry.update();
        }
    }
}

// an auton written to help out a rookie team without programming experience - NOT FOR OUR ROBOT

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonSample extends LinearOpMode {

    DcMotor driveLeft;
    DcMotor driveRight;

    Servo servoOne;
    Servo servoTwo;

    DcMotor arm;

    @Override
    public void runOpMode() throws InterruptedException {

        driveLeft = hardwareMap.get(DcMotor.class, "driveLeft"); // change driveLeft to name of motor
        driveRight = hardwareMap.get(DcMotor.class, "driveRight"); // change driveRight to name of motor

        servoOne = hardwareMap.get(Servo.class, "servoOne"); // change servoOne to name of servo

        arm = hardwareMap.get(DcMotor.class, "arm"); // change arm to name of arm

        waitForStart();

        if (opModeIsActive()) {
            // drive forwards for 1 second
            driveRight.setPower(1);
            driveLeft.setPower(1);
            sleep(1000);
            driveLeft.setPower(0);
            driveRight.setPower(0);

            // drive arm for half a second
            arm.setPower(1);
            sleep(500);
            arm.setPower(0);

            // move servos 90°
            servoOne.setPosition(servoOne.getPosition() + .5);

        }

    }

}

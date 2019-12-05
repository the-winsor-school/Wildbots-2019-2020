package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "Current TeleOp", group = "Testing")
public class CurrentTeleOp extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    double servoPos = .1;

    DcMotor grabArm;
    Servo grabHand;
    DcMotor dragArm;

    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        grabArm = hardwareMap.get(DcMotor.class, "grabArm");
        grabHand = hardwareMap.get(Servo.class, "grabHand");
        dragArm = hardwareMap.get(DcMotor.class, "baseArm");

        waitForStart();

        while(opModeIsActive()) {

            //change driving mode
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

            // gamepad 2 up/down to move grab arm up/down
            if (gamepad2.dpad_up){
                grabArm.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                grabArm.setPower(-1);
            }
            else {
                grabArm.setPower(0);
            }

            // gamepad 2 left right to open/clos\\e the grab hand
            grabHand.setPosition(servoPos);
            if (gamepad2.dpad_left) {
                servoPos += .001;
            }
            else if (gamepad2.dpad_right && servoPos > 0) {
                servoPos -= .001;
            }

            telemetry.addData("Servo position: ", servoPos);

            // gamepad 2 a b to flip the drag arm
            if (gamepad2.a) {
                dragArm.setPower(1);
            }
            else if (gamepad2.b) {
                dragArm.setPower(-.5);
            }
            else {
                dragArm.setPower(0);
            }

            telemetry.update();

        }
    }

}
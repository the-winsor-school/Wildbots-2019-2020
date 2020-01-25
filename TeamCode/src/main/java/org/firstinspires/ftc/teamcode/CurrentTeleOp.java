//just a note - the robot is called R2B2 but Arby for short (RB) RB we have the beeps

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name = "Bevel and Drag TeleOp")
public class CurrentTeleOp extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    double servoPos = 0.3; //init servos

    DcMotor grabArm;
    Servo grabHand;
    Servo dragNoYoda;
    Servo dragYoda;

    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        //adjusting strafe bias because FR wheel moves slower

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        grabArm = hardwareMap.get(DcMotor.class, "grabArm");
        grabHand = hardwareMap.get(Servo.class, "grabHand");
        dragNoYoda = hardwareMap.get(Servo.class, "dragLeft");
        dragYoda = hardwareMap.get(Servo.class, "dragRight");

        waitForStart();

        while(opModeIsActive()) {

            //change driving mode
            if (gamepad1.b) {
                drivingMode++;
                drivingMode %= DrivingMode.values().length;
                drivingLibrary.setMode(drivingMode);
            }
            //strafe

            if (gamepad1.y) {
                drivingLibrary.setSpeed(.5);
            }

            //drivingLibrary.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            drivingLibrary.bevelDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", drivingLibrary.getMode());

            // gamepad 2 up/down to move grab arm up/down
            if (gamepad2.dpad_up){
                grabArm.setPower(-1);
            }
            else if (gamepad2.dpad_down) {
                grabArm.setPower(1);
            }
            else {
                grabArm.setPower(0);
            }

            // gamepad 2 left right to open/close the grab hand
            grabHand.setPosition(servoPos);
            if (gamepad2.y && servoPos < 1) {
                servoPos +=.1;
            }
            else if (gamepad2.x && servoPos > 0) {
                servoPos -= .1;
            }

            telemetry.addData("Servo position: ", servoPos);

            //set drag servo positions
            //go up when a is pressed
            //go down when b is pressed
            if (gamepad2.a) {
                dragNoYoda.setPosition(.1);
                dragYoda.setPosition(.95);
            }
            else if (gamepad2.b) {
                dragNoYoda.setPosition(.85);
                dragYoda.setPosition(.2);
            }


            //telemetry.addData("Motor powers", drivingLibrary.getMotorPower());
            telemetry.addData("No yoda servo position", dragNoYoda.getPosition());
            telemetry.addData("Yoda servo position", dragYoda.getPosition());
            telemetry.update();

        }
    }

}

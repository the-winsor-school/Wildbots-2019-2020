package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Testing", group = "Testing")
@Disabled

public class ArmTesting extends LinearOpMode {
    DcMotor grabArm;
    DcMotor baseArm;
    Servo grabHand;

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        grabHand = hardwareMap.get(Servo.class, "grabHand");
        grabArm = hardwareMap.get(DcMotor.class, "grabArm");
        double servoPos = 0.5;

        while (opModeIsActive()){
            //grab arm - the one that holds the 🅱️loccs


            if (gamepad1.x){
                grabArm.setPower(0.7);
            }
            else if (gamepad1.y) {
                grabArm.setPower(-0.7);
            }
            else {
                grabArm.setPower(0);
            }

            // base arm - the one that moves the 🅱️ase
            baseArm = hardwareMap.get(DcMotor.class, "baseArm");

            if (gamepad1.a){
                baseArm.setPower(0.3);

            }
            else if (gamepad1.b){
                baseArm.setPower(-0.3);
            }

            else{
                baseArm.setPower(0);

            }


            //grab hand - the one that open/close around le 🅱️loccs




            if (gamepad1.dpad_left) {
                servoPos += .001;
                grabHand.setPosition(servoPos);
            }

            else if (gamepad1.dpad_right) {
                servoPos -= .001;
                grabHand.setPosition(servoPos);
            }


            telemetry.addData( "Status:", "Running");

            telemetry.update();

        }

    }
}

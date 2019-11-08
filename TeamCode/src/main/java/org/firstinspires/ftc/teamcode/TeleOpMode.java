package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@TeleOp(name  = "TeleOp Mode", group = "Finished")
public class TeleOpMode extends LinearOpMode {
    //drive train
    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor grabArm;
    Servo baseArm;

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



            //grab arm

            grabArm = hardwareMap.get(DcMotor.class, "grabArm");

            if (gamepad1.x){
                grabArm.setPower(0.3);
            }
            if (gamepad1.y) {
                grabArm.setPower(-0.3);
            }

            // base arm
            baseArm = hardwareMap.get(Servo.class, "baseArm");
            double baseArmPos = 0.5;

            //move base arm??
            if (gamepad1.a){
                baseArm.setPosition(baseArmPos);
                baseArmPos += 0.5;

            }
            if (gamepad1.b){
                baseArm.setPosition(baseArmPos);
                baseArmPos -=0.5;
            }

            else{


            }
            telemetry.addData( "Status:", "Running");

            telemetry.update();





        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test New Motor", group = "Testing")
@Disabled

public class NewMotorTest extends LinearOpMode {
    DcMotor motor;

    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        motor = hardwareMap.get(DcMotor.class, "motor");
        double servoPos = 0.5;

        while (opModeIsActive()){
            if (gamepad1.x){
                motor.setPower(1);
            }
            else if (gamepad1.y) {
                motor.setPower(1);
            }
            else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setPower(0);
            }

            telemetry.addData( "Status:", "Running");

            telemetry.update();

        }

    }
}

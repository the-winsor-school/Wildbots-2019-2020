package org.firstinspires.ftc.teamcode_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mreeves on 4/26/19.
 */

@TeleOp
public class ExampleOpMode extends LinearOpMode {
    DcMotor motor;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();

        while(opModeIsActive()){
            motor.setPower(-gamepad1.left_stick_y);

            if(gamepad1.a) {
                servo.setPosition(0);

            }

            if(gamepad2.b) {
                servo.setPosition(1);
            }
        }
    }
}

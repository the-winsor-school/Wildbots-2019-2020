package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous
public class GrabberTest extends LinearOpMode {
    Servo grabber;
    @Override
    public void runOpMode() throws InterruptedException {
        grabber = hardwareMap.get(Servo.class, "grabber");
        boolean ranOnce = false;
        waitForStart();
        if (opModeIsActive()) {
            grabber.setPosition(1);
            sleep(1875);
            grabber.setPosition(0.3);
            sleep(1875);
            grabber.setPosition(1);
        }
    }
}

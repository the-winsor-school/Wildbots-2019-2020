package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.libraries.DrivingLibrary;

import java.util.Date;

/**
 * Created by lamanwyner on 2/8/19.
 */

@TeleOp(name = "Test Values for Simple Mineral Sampling", group = "Test")
public class TestMineralSampling extends LinearOpMode {
    DrivingLibrary drivingLibrary;

    Date date; // for getting current time
    long time;
    boolean clockRunning;
    long clockTimeMillis;

    boolean aPressed;

    @Override
    public void runOpMode() throws InterruptedException {
        //drivingLibrary = new DrivingLibrary(this);
        date = new Date();
        time = date.getTime(); // returns current date/time in milliseconds
        clockTimeMillis = 0;

        aPressed = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !aPressed) {
                clockRunning = !clockRunning;
                time = date.getTime();
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            if (clockRunning) {
                telemetry.addData("Clock", "running");
                clockTimeMillis = date.getTime() - time;
            }

            telemetry.addData("Time", convertMillis(clockTimeMillis));
            telemetry.update();
        }
    }

    private String convertMillis(long ts) {
        int millis = (int)(ts % 1000);
        ts /= 1000;
        int secs = (int)(ts % 60);
        if (time/60 > 0) {
            return (time/60) + ":" + secs + "." + millis;
        }
        return secs + "." + millis;
    }
}

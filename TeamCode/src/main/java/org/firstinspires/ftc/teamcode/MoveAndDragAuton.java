package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous

public class MoveAndDragAuton extends LinearOpMode {
    DrivingLibrary drivingLibrary;
    int drivingMode;
    Servo baseArm;

    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        boolean ranOnce = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){
                baseArm = hardwareMap.get(Servo.class, "baseArm");
                drivingLibrary.strafe(0,0.5f); //go straight
                sleep(700);
                drivingLibrary.brakeStop();
                drivingLibrary.turn(1,0);
                sleep(500);
                baseArm.setPosition(0.5);
                drivingLibrary.strafe(0,-0.5f); //go straight
                sleep(200);

                baseArm.setPosition(-0.5);
            }
            ranOnce = true;
        }
    }
}

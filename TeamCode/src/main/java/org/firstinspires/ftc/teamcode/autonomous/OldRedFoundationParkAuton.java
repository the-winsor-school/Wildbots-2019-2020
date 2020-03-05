package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous
@Disabled
public class OldRedFoundationParkAuton extends LinearOpMode {
    DrivingLibrary drivingLibrary;
    int drivingMode;

    Servo dragNoYoda;
    Servo dragYoda;

    boolean ranOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        dragNoYoda = hardwareMap.get(Servo.class, "dragLeft");
        dragYoda = hardwareMap.get(Servo.class, "dragRight");

        waitForStart();

        if (opModeIsActive()) {
            if (!ranOnce) {
                //positive y value to drive backwards
                drivingLibrary.bevelDrive(0, .75f, 0);
                sleep(825);
                drivingLibrary.brakeStop();
                //strafe towards wall
                drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(1000);
                drivingLibrary.brakeStop();
                //grab foundation
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(1500);
                //strafe away from wall
                /*drivingLibrary.bevelDrive(-.25f, 0, 0);
                sleep(250);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(250);
                drivingLibrary.brakeStop();*/
                //negative y value to drive forwards
                drivingLibrary.bevelDrive(0, -.5f, 0);
                sleep(250);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(250);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(250);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(250);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                drivingLibrary.brakeStop();
                sleep(100);
                drivingLibrary.spinToAngle(-Math.PI/2 - .1);
                sleep(500);
                drivingLibrary.drive(0, .5f, 0);
                sleep(1500);
                dragNoYoda.setPosition(.1);
                dragYoda.setPosition(.95);
                //strafe away from foundation
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(250);
                drivingLibrary.brakeStop();
                //strafe into the wall at half speed because we are nice yay
                drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(1000);
                drivingLibrary.brakeStop();
                //back up to parking spot
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(750);
                drivingLibrary.brakeStop();
                ranOnce = true;
            }
        }
    }

}

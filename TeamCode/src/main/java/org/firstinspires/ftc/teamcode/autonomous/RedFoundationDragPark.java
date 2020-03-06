package org.firstinspires.ftc.teamcode.autonomous;
//this works!!! put the robot on the side close to the wall

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous
public class RedFoundationDragPark extends LinearOpMode {
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
                drivingLibrary.bevelDrive(-.25f, 0, 0);
                sleep(500);
                drivingLibrary.brakeStop();

                //negative y value to drive forwards
                drivingLibrary.bevelDrive(0, -.5f, 0);
                sleep(1200);
                drivingLibrary.brakeStop();

                //spins the foundation into the ZONE
                drivingLibrary.spinToAngle(-Math.PI/2 - .1);
                sleep(500);

                //unhooks the drag things from the foundation
                dragNoYoda.setPosition(.1);
                dragYoda.setPosition(.95);

                //strafe away from foundation
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(1000);

                //and parks
            }
        }
    }

}

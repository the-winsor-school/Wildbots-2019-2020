package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous

public class MoveAndParkAuton extends LinearOpMode {
    DrivingLibrary drivingLibrary;
    int drivingMode;

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
                drivingLibrary.strafe(0,0.5f);
                sleep(300);
                drivingLibrary.brakeStop();
            }
            ranOnce = true;
        }


    }
}

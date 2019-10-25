package org.firstinspires.ftc.teamcode_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.libraries.DrivingLibrary;

/**
 * Created by mreeves on 4/26/19.
 */
@Autonomous
public class ExampleAuton extends LinearOpMode {

    DrivingLibrary drivingLibrary;

    @Override
    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(.5);
        waitForStart();

        while (opModeIsActive()){
            drivingLibrary.strafe(0,1);
            sleep(2000);
            drivingLibrary.turn(-1,0);
            sleep(1000);
            drivingLibrary.brakeStop();
        }

    }
}

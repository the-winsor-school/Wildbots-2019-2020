package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous

public class SimpleAutonWithEncoders extends LinearOpMode {
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
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENDODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){

            }
            ranOnce = true;
        }
    }
}

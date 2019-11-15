package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class SimpleAutonWithEncoders extends LinearOpMode {
    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor leftRear;


    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);
        boolean ranOnce = false;
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){

            }
            ranOnce = true;
        }
    }
}

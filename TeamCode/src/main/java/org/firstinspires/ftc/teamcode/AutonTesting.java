package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.List;

@Autonomous(name = "Auton Testing", group = "testing")
public class AutonTesting extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    OpenCvCamera phoneCam;
    OpenCVTestTwo.StageSwitchingPipeline stageSwitchingPipeline;

    Rev2mDistanceSensor distanceSensor;

    String identity;

    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new OpenCVTestTwo.StageSwitchingPipeline();
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);

        boolean ranOnce = false;
        boolean skystoneFound = false;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (!ranOnce) {
                drivingLibrary.resetEncoderValues();
                double dist = distanceSensor.getDistance(DistanceUnit.CM);
                double initialDist = dist;
                telemetry.addData("starting distance", initialDist);
                while (dist > 15) {
                    double motorPower = dist / initialDist / 2;
                    drivingLibrary.bevelDrive(0, (float) motorPower, 0);
                    dist = distanceSensor.getDistance(DistanceUnit.CM);
                    telemetry.addData("motor powers", drivingLibrary.getMotorPower());
                    telemetry.update();
                }
                drivingLibrary.brakeStop();
                sleep(500);
                dist = distanceSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("ending distance", dist);
                telemetry.addData("number of yellow blobs", stageSwitchingPipeline.getNumContoursFound());
                while (skystoneFound == false) {
                    if (stageSwitchingPipeline.getNumContoursFound() == 0) {
                        identity = "skystone";
                        drivingLibrary.brakeStop();
                        skystoneFound = true;
                    }
                    else {
                        drivingLibrary.bevelDrive(.5f, 0f, 0f);
                    }
                }
                telemetry.addData("stone or skystone?", identity);
                telemetry.update();
                ranOnce = true;
            }
        }
    }
}

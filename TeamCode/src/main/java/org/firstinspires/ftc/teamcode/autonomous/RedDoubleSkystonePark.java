package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.OpenCVTestTwo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class RedDoubleSkystonePark extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor grabArm;

    Servo grabHand;

    OpenCvCamera phoneCam;
    OpenCVTestTwo.StageSwitchingPipeline stageSwitchingPipeline;

    Rev2mDistanceSensor stoneDistSensor;

    String identity;


    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        stoneDistSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDist");

        grabArm = hardwareMap.get(DcMotor.class, "grabArm");

        grabHand = hardwareMap.get(Servo.class, "grabHand");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new OpenCVTestTwo.StageSwitchingPipeline();
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        boolean ranOnce = false;
        boolean skystoneFound = false;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            if (!ranOnce) {
                //positive y drives backwards
                //negative y drives forwards
                //positive x value drives left
                //negative x value drives right
                drivingLibrary.resetEncoderValues();
                //drive to stones
                double stoneDist = this.stoneDistSensor.getDistance(DistanceUnit.CM);
                double initialDist = stoneDist;
                while (stoneDist > 22) {
                    double motorPower = stoneDist / initialDist / 2;
                    drivingLibrary.bevelDrive(0, (float) -motorPower, 0);
                    stoneDist = this.stoneDistSensor.getDistance(DistanceUnit.CM);
                }
                drivingLibrary.brakeStop();

                sleep(500);
                stoneDist = this.stoneDistSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("ending distance", stoneDist);
                telemetry.addData("number of yellow blobs", stageSwitchingPipeline.getNumContoursFound());
                //strafe until skystone is found
                while (!skystoneFound) {
                    if (stageSwitchingPipeline.getNumContoursFound() == 0) {
                        identity = "skystone";
                        drivingLibrary.brakeStop();
                        skystoneFound = true;
                    }
                    else {
                        drivingLibrary.bevelDrive(.5f, 0f, 0f);
                    }
                }
                //flip arm, grab block
                grabHand.setPosition(.7);
                for (float i = -.4f; i > -1; i-= .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabArm.setPower(-1);
                sleep(2000);
                grabArm.setPower(0);
                grabHand.setPosition(0);
                sleep(500);
                grabArm.setPower(1);
                sleep(400);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.bevelDrive(0, 1, 0);
                sleep(200);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to the other side
                drivingLibrary.spinToAngle(-Math.PI/2);
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1700);
                drivingLibrary.brakeStop();
                //release stone
                grabHand.setPosition(.7);
                //drive back to the other side
                drivingLibrary.drive(0, .75f, 0);
                sleep(3000);
                drivingLibrary.brakeStop();
                //turn back
                drivingLibrary.spinToAngle(0.0001);
                //drives forward a bit
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(200);
                drivingLibrary.brakeStop();
                //tries to detect skystones
                while (!skystoneFound) {
                    if (stageSwitchingPipeline.getNumContoursFound() == 0) {
                        identity = "skystone";
                        drivingLibrary.brakeStop();
                        skystoneFound = true;
                    }
                    else {
                        drivingLibrary.bevelDrive(-.5f, 0f, 0f);
                    }
                }
                //flip arm, grab block
                grabHand.setPosition(.7);
                for (float i = -.4f; i > -1; i-= .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabArm.setPower(-1);
                sleep(2000);
                grabArm.setPower(0);
                grabHand.setPosition(0);
                sleep(500);
                grabArm.setPower(1);
                sleep(400);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.bevelDrive(0, 1, 0);
                sleep(200);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to the other side
                drivingLibrary.spinToAngle(-Math.PI/2);
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1000);
                drivingLibrary.brakeStop();
                //release stone
                grabArm.setPower(-1);
                sleep(350);
                grabArm.setPower(0);
                grabHand.setPosition(.7);
                //drive back to the line and parks
                drivingLibrary.drive(0, .75f, 0);
                sleep(500);
                drivingLibrary.brakeStop();
                ranOnce = true;
            }
        }
    }
}

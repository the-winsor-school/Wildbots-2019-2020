package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Blue: Skystone, Drag, Park")
public class BlueSkystoneDragPark extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor dragArm;
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

        stoneDistSensor = hardwareMap.get(Rev2mDistanceSensor.class, "stoneDistanceSensor");

        dragArm = hardwareMap.get(DcMotor.class, "dragArm");
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

        while (opModeIsActive()) {
            if (!ranOnce) {
                drivingLibrary.resetEncoderValues();
                //drive to stones
                double stoneDist = this.stoneDistSensor.getDistance(DistanceUnit.CM);
                double initialDist = stoneDist;
                while (stoneDist > 32) {
                    double motorPower = stoneDist / initialDist / 2;
                    drivingLibrary.drive(0, (float) -motorPower, 0);
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
                        drivingLibrary.drive(.5f, 0f, 0f);
                    }
                }
                //flip arm, grab block
                grabHand.setPosition(.7);
                grabArm.setPower(-1);
                sleep(5000);
                grabArm.setPower(0);
                grabHand.setPosition(0);
                grabArm.setPower(1);
                sleep(500);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.drive(0, .5f, 0);
                sleep(350);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to foundation, then turn back
                drivingLibrary.spinToAngle(Math.PI/2 - .15);
                drivingLibrary.drive(0, -1, 0);
                sleep(1750);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(0);
                //raise arm a bit, then drive forward
                grabArm.setPower(1);
                sleep(1000);
                grabArm.setPower(0);
                drivingLibrary.drive(0, -.5f, 0);
                sleep(550);
                drivingLibrary.brakeStop();
                //lower arm, release stone
                grabArm.setPower(-1);
                sleep(1000);
                grabArm.setPower(0);
                grabHand.setPosition(.7);
                //raise arm a bit
                grabArm.setPower(1);
                sleep(1000);
                grabArm.setPower(0);
                drivingLibrary.drive(0, .5f, 0);
                sleep(350);
                drivingLibrary.brakeStop();
                //180° to use drag arm
                drivingLibrary.spinToAngle(Math.PI);
                drivingLibrary.drive(0, .5f, 0);
                sleep(350);
                drivingLibrary.brakeStop();
                //strafe into the wall basically
                drivingLibrary.drive(.5f, 0, 0);
                sleep(1000);
                drivingLibrary.brakeStop();
                //return the grab arm to its proper place
                grabArm.setPower(1);
                sleep(3500);
                grabArm.setPower(0);
                //actiave the drag arm
                dragArm.setPower(1);
                sleep(250);
                //drive back with the drag arm
                drivingLibrary.drive(0,-1, 0);
                sleep(3500);
                drivingLibrary.brakeStop();
                dragArm.setPower(0);
                //flip the drag arm back in
                dragArm.setPower(-1);
                sleep(500);
                dragArm.setPower(0);
                //park
                drivingLibrary.drive(-.5f, 0, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                ranOnce = true;
            }
        }
    }
}

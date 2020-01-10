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

@Autonomous(name = "Auton Testing", group = "testing")
public class AutonTesting extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor dragArm;
    DcMotor grabArm;

    Servo grabHand;

    OpenCvCamera phoneCam;
    OpenCVTestTwo.StageSwitchingPipeline stageSwitchingPipeline;

    Rev2mDistanceSensor stoneDistSensor;
    Rev2mDistanceSensor foundationDistSensor;

    String identity;


    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        stoneDistSensor = hardwareMap.get(Rev2mDistanceSensor.class, "stoneDistanceSensor");
        foundationDistSensor = hardwareMap.get(Rev2mDistanceSensor.class, "foundationDistanceSensor");

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
                        drivingLibrary.drive(-.5f, 0f, 0f);
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
                drivingLibrary.drive(0, .5f, 0);
                sleep(350);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(Math.PI/2 - .1);
                drivingLibrary.drive(0, -1, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(-Math.PI/2 + .1);
                grabArm.setPower(1);
                sleep(1000);
                grabArm.setPower(0);
                drivingLibrary.drive(0, .5f, 0);
                sleep(100);
                drivingLibrary.brakeStop();
                grabHand.setPosition(.7);

                //outline for the remainder of autonomous
                /*double foundationDist = this.foundationDistSensor.getDistance(DistanceUnit.CM);
                while (foundationDist > 45) {
                    drivingLibrary.drive(.5f, 0, 0);
                    foundationDist = this.foundationDistSensor.getDistance(DistanceUnit.CM);
                }
                drivingLibrary.brakeStop();*/
                //place the skystone !
                //bring the grab arm back up enough that the drag arm can flip out
                /*drivingLibrary.spinToAngle(Math.PI);
                dragArm.setPower(0.5);
                sleep(2000);
                drivingLibrary.drive(0f,-.5f, 0f);
                sleep(5500);
                drivingLibrary.brakeStop();
                dragArm.setPower(0);
                sleep(2000);
                dragArm.setPower(-0.5);
                sleep(500);
                dragArm.setPower(0);
                drivingLibrary.drive(-.5f, 0, 0);
                sleep(1500);
                drivingLibrary.brakeStop();*/
                ranOnce = true;
            }
        }
    }
}

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
public class BlueSkystonePark extends LinearOpMode {

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
                while (stoneDist > 25) {
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
                sleep(450);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.bevelDrive(0, 1, 0);
                sleep(100);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to foundation then start slowing down
                drivingLibrary.spinToAngle(Math.PI/2);
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1125);
                drivingLibrary.bevelDrive(0, -.5f, 0);
                grabArm.setPower(1);
                sleep(400);
                grabArm.setPower(0);
                sleep(1250);
                drivingLibrary.brakeStop();
                sleep(250);
                grabHand.setPosition(.7);
                sleep(250);
                //park in the far spot
                drivingLibrary.bevelDrive(-.75f, 0, 0);
                sleep(280);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, .75f, .03f);
                grabArm.setPower(-1);
                sleep(400);
                grabArm.setPower(0);
                sleep(850);
                drivingLibrary.brakeStop();
                //park in the close spot if we need to
                /* drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(75);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0, .75f, .03f);
                sleep(1250);
                drivingLibrary.brakeStop(); */


                /*grabArm.setPower(1);
                sleep(325);
                grabArm.setPower(0);
                drivingLibrary.spinToAngle(.0001);
                //then drive forward
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(325);
                drivingLibrary.brakeStop();
                //lower arm, release stone
                //grabArm.setPower(-1);
                //sleep(350);
                //grabArm.setPower(0);
                grabHand.setPosition(.7);
                //raise arm a bit
                //grabArm.setPower(1);
                //sleep(500);
                //grabArm.setPower(0);
                //drive back then park and move arm down a little
                drivingLibrary.bevelDrive(0, .75f, 0);
                sleep(250);
                drivingLibrary.brakeStop();
                drivingLibrary.spinToAngle(-Math.PI/2 + .13);
                sleep(200);
                drivingLibrary.bevelDrive(0, .75f, 0);
                grabArm.setPower(-1);
                sleep(200);
                grabArm.setPower(0);
                sleep(900);
                drivingLibrary.brakeStop();*/
                ranOnce = true;
            }
        }
    }
}

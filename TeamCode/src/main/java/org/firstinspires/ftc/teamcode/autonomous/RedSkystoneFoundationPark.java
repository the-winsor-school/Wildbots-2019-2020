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

@Autonomous(name= "RED SKYSTONE FOUNDATION PARK")
public class RedSkystoneFoundationPark extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor grabArm;

    Servo grabHand;

    Servo dragNoYoda;
    Servo dragYoda;

    OpenCvCamera phoneCam;
    OpenCVTestTwo.RedStageSwitchingPipeline stageSwitchingPipeline;

    Rev2mDistanceSensor stoneDistSensor;

    String identity;

    int skyPosCounter = 0;

    int skyPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        //use "leftDist" for opmodes on the right side!
        stoneDistSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rightDist");

        grabArm = hardwareMap.get(DcMotor.class, "grabArm");

        grabHand = hardwareMap.get(Servo.class, "grabHand");

        dragNoYoda = hardwareMap.get(Servo.class, "dragLeft");
        dragYoda = hardwareMap.get(Servo.class, "dragRight");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new OpenCVTestTwo.RedStageSwitchingPipeline();
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);

        boolean ranOnce = false;
        boolean skystoneFound = false;

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            if (!ranOnce) {
                /*//positive y drives backwards
                //negative y drives forwards
                //positive x value drives left
                //negative x value drives right
                drivingLibrary.resetEncoderValues();
                //drive to stones
                double stoneDist = this.stoneDistSensor.getDistance(DistanceUnit.CM);
                double initialDist = stoneDist;
                while (stoneDist > 15) {
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
                    skyPosCounter += 1;
                }
                telemetry.addData("Loop Iterations: ", skyPosCounter);
                if (skyPosCounter <= 1) {
                    skyPosition = 1;
                }
                else if (skyPosCounter <= 42) {
                    skyPosition = 2;
                }
                else {
                    skyPosition = 3;
                }
                telemetry.addData("Skystone Position: ", skyPosition);
                telemetry.update();
                //flip arm, grab block
                grabHand.setPosition(.7);
                for (float i = -.4f; i > -1; i-= .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabArm.setPower(-1);
                sleep(2000);
                grabArm.setPower(0);
                sleep(500);
                grabHand.setPosition(0);
                sleep(500);
                grabArm.setPower(1);
                sleep(400);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.bevelDrive(0, 1, 0);
                sleep(100);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to other side of field, raise arm a bit, then turn back
                drivingLibrary.spinToAngle(-Math.PI/2);
                //drive forwards to other side of the field based on skystone position
                drivingLibrary.bevelDrive(0, -1, 0);
                switch (skyPosition) {
                    case 1:
                        sleep(1700);
                        break;
                    case 2:
                        sleep(2000);
                    break;
                    case 3:
                        sleep(2050);
                    break;
                    default:
                        sleep(1875);
                    break;
                }
                drivingLibrary.brakeStop();
                grabArm.setPower(1);
                sleep(350);
                grabArm.setPower(0);
                drivingLibrary.spinToAngle(-.0001);
                //then drive forward
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(150);
                drivingLibrary.brakeStop();
                //lower arm, release stone
                grabArm.setPower(-1);
                sleep(425);
                grabArm.setPower(0);
                grabHand.setPosition(.7);
                //raise arm a bit
                grabArm.setPower(1);
                sleep(650);
                grabArm.setPower(0);
                drivingLibrary.bevelDrive(0, .7f,0);
                sleep(300);
                drivingLibrary.brakeStop();
                //option 1 - drag
                drivingLibrary.spinToAngle(-Math.PI);
                //lower arm with acceleration, close hand, strafe towards wall
                for (float i = .4f; i < 1; i += .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabHand.setPosition(0);
                grabArm.setPower(1);
                drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(800);
                grabArm.setPower(0);
                sleep(200);*/
                //positive y value to drive backwards
                //strafe towards wall
                drivingLibrary.spinToAngle(Math.PI/2);
                drivingLibrary.spinToAngle(0);
                drivingLibrary.spinToAngle(Math.PI);
                drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(1000);
                drivingLibrary.brakeStop();
                //grab foundation
                dragNoYoda.setPosition(.85);
                dragYoda.setPosition(.1);
                sleep(1500);
                //negative y value to drive forwards
                drivingLibrary.bevelDrive(0, -.5f, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                sleep(100);
                drivingLibrary.spinToAngle(-Math.PI/2 - .1);
                sleep(500);
                drivingLibrary.drive(0, .5f, -.25f);
                sleep(1500);
                dragNoYoda.setPosition(.1);
                dragYoda.setPosition(.95);
                //strafe away from foundation
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(250);
                drivingLibrary.brakeStop();
                //strafe into the wall at half speed because we are nice yay
                /*drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(1000);
                drivingLibrary.brakeStop();*/
                //back up to parking spot
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(750);
                drivingLibrary.brakeStop();

                ranOnce = true;
            }
        }
    }
}

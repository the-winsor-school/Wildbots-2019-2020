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

@Autonomous(name= "BLUE SKYSTONE ON FOUNDATION")
public class BlueSkystoneOnFoundation extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    DcMotor grabArm;

    Servo grabHand;

    Servo dragNoYoda;
    Servo dragYoda;

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

        dragNoYoda = hardwareMap.get(Servo.class, "dragLeft");
        dragYoda = hardwareMap.get(Servo.class, "dragRight");

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
                while (stoneDist > 28) {
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
                sleep(500);
                grabHand.setPosition(0);
                sleep(1000);
                grabArm.setPower(1);
                sleep(400);
                grabArm.setPower(0);
                //drive back a bit
                drivingLibrary.bevelDrive(0, 1, 0);
                sleep(100);
                drivingLibrary.brakeStop();
                //turn, then drive forwards to other side of field, raise arm a bit, then turn back
                drivingLibrary.spinToAngle(Math.PI/2);
                drivingLibrary.bevelDrive(0, -1, 0);
                sleep(1875);
                drivingLibrary.brakeStop();
                grabArm.setPower(1);
                sleep(200);
                grabArm.setPower(0);
                drivingLibrary.spinToAngle(.0001);
                //then drive forward
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(700);
                drivingLibrary.brakeStop();
                //lower arm, release stone
                grabArm.setPower(-1);
                sleep(425);
                grabArm.setPower(0);
                grabHand.setPosition(.7);
                //raise arm a bit
                grabArm.setPower(1);
                sleep(500);
                grabArm.setPower(0);
                drivingLibrary.bevelDrive(0, .75f,0);
                sleep(350);
                drivingLibrary.brakeStop();
                //option 1 - drag
                drivingLibrary.spinToAngle(Math.PI);
                for (float i = .4f; i < 1; i += .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabArm.setPower(1);
                sleep(1000);
                grabArm.setPower(0);
                drivingLibrary.bevelDrive(0, .75f, 0);
                sleep(350);
                drivingLibrary.brakeStop();
                sleep(500);
                dragNoYoda.setPosition(1);
                dragYoda.setPosition(0);
                sleep(1000);
                drivingLibrary.brakeStop();
                drivingLibrary.bevelDrive(0,-.4f,0);
                sleep(5000);
                drivingLibrary.brakeStop();

                //drag end

                //option 2 - park
                /*drivingLibrary.spinToAngle(Math.PI/2);
                drivingLibrary.bevelDrive(0, .5f, 0);
                sleep(950);
                drivingLibrary.brakeStop();*/
                //park end

                /*drivingLibrary.bevelDrive(0, .75f, 0);
                sleep(300);
                drivingLibrary.brakeStop();
                //spin 180°
                drivingLibrary.spinToAngle(Math.PI);
                //move arm down
                grabArm.setPower(-1);
                sleep(250);
                grabArm.setPower(0);
                drivingLibrary.bevelDrive(0, .75f, -.01f);
                sleep(125);
                drivingLibrary.brakeStop();*/
                /*//literally copied and pasted foundation drag auton
                //strafe toward wall
                drivingLibrary.bevelDrive(.5f, 0, 0);
                sleep(450);
                drivingLibrary.brakeStop();
                //grab foundation
                dragNoYoda.setPosition(.85);
                dragYoda.setPosition(.2);
                sleep(1000);
                drivingLibrary.bevelDrive(0, -.5f, 0);
                sleep(1500);
                drivingLibrary.brakeStop();
                sleep(100);
                drivingLibrary.bevelDrive(0, 0, -.35f);
                sleep(3200);
                drivingLibrary.brakeStop();
                drivingLibrary.drive(0, .5f, 0);
                sleep(1500);
                dragNoYoda.setPosition(.1);
                dragYoda.setPosition(.95);
                //strafe away from foundation
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(250);
                drivingLibrary.brakeStop();
                //strafe sideways at half speed because we are nice yay
                drivingLibrary.bevelDrive(-.5f, 0, 0);
                sleep(500);
                drivingLibrary.brakeStop();
                //back up to parking spot
                drivingLibrary.bevelDrive(0, -.75f, 0);
                sleep(750);
                drivingLibrary.brakeStop();*/
                ranOnce = true;
            }
        }
    }
}

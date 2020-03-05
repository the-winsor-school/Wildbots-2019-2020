package org.firstinspires.ftc.teamcode_2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.libraries.DrivingLibrary;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lamanwyner on 1/25/19.
 */

@Disabled
@TeleOp(name = "Test PID Constants for Mineral Sampling", group = "Test")
public class TestPID extends LinearOpMode {
     // This OpMode is for testing the constant values in the PID loop using a game controller

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AYOTCgT/////AAABmewkUXExAUVzskAoLuha7w9lfk/CuSoaWXjaMtTsIKpGAyXJ+18HzPsnSewFBAtMzZABHnGc3ojJimkfSVONSkh59LkxPebzR34qCnDr+m1ybQeVjTRobnNZts+W/tgDDMAnbCEsOpI8nQuCjPwUBBTm6SkS8ApJ4eTrXLlsSKVwQ8y7X1LNCS1rA2U9PevNBiiu5sI76rLIijmZ72iifKgHnPjDLWHJqPI+a1dOcx9L0+2L4KqTC+iX3W1Y31D5IXtSJU9bSIAnA0SaWqRDfiRaSre8PU7GW14cfeXj/YnHz18mM2KIaytZbmiXx2s9GNTd+6DAwhxE081eYEN0ecggbEh2TvoZT/BtmCqvPq35";

    //private VuforiaLocalizer vuforia; // Vuforia Library gives opMode easy access to the phone camera
    //private TFObjectDetector tfod; // An object that contains a trained model and builtin functions for identifying game elements

    private DrivingLibrary drivingLibrary; // a class containing simple drive train functions

    @Override
    public void runOpMode() throws InterruptedException {
        //initVuforia();

        /*if (ClassFactory.getInstance().canCreateTFObjectDetector()) { // Need to check because older phones are not
            initTfod();                                               // compatible with tensorflow
        } else {
            telemetry.addLine("Sorry! This device is not compatible with TFOD");
            telemetry.update();
            stop();
        }*/

        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(.5);

        waitForStart();

        if (opModeIsActive()) {
            //tfod.activate();
            float diff = 0, totalDiff = 0, lastDiff = 0;        // the integral and derivative parts of this controller use previous error
                                                      // values and therefore need to be declared outside the while loop

            float[] pidVals = new float[] {0, 0, 0};  // stores the constants for the PID controller
            String[] pidNames = new String[] {"P", "I", "D"}; // used for telemetry display (not crucial to algorithm)
            int pidInd = 0; // indicates which constant user is currently changing

            boolean upPressed = false;
            boolean downPressed = false;
            boolean leftPressed = false;
            boolean rightPressed = false;
            boolean initPID = false;

            boolean runOnce = false;

            while (opModeIsActive() && !runOnce) {
                telemetry.addLine("Change the PID values using the up and down buttons on the controller. Switch between variables using the left and right arrows. The currently edited variable will appear on the screen.");
                telemetry.addData("Currently changing", pidNames[pidInd]);
                if (gamepad1.dpad_up && !upPressed) { // increment value of constant
                    pidVals[pidInd] += 0.01;
                } else if (gamepad1.dpad_down && !downPressed) {
                    pidVals[pidInd] -= 0.01;
                } else if (gamepad1.dpad_left && !leftPressed) { // switch between constants
                    pidInd--;
                    pidInd += 3;
                    pidInd %= 3;
                } else if (gamepad1.dpad_right && !rightPressed) {
                    pidInd++;
                    pidInd %= 3;
                }

                upPressed = gamepad1.dpad_up;
                downPressed = gamepad1.dpad_down;
                leftPressed = gamepad1.dpad_left;
                rightPressed = gamepad1.dpad_right;

                // display PID values
                telemetry.addData("P", pidVals[0]);
                telemetry.addData("I", pidVals[1]);
                telemetry.addData("D", pidVals[2]);

                if (gamepad1.a) {
                    initPID = true;
                }
                telemetry.update();

                //List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //ArrayList<Recognition> goldMinerals = new ArrayList<>(); // store gold mineral recognitions in a list for later
                                                                         // retrieval
                /**if (updatedRecognitions != null) {
                    for (Recognition r : updatedRecognitions) {
                        if (r.getLabel() == LABEL_GOLD_MINERAL) {
                            goldMinerals.add(r);
                        }
                    }

                    telemetry.addData("Gold Minerals Detected", goldMinerals.size());

                    if (goldMinerals.size() == 1) {**/
                if (initPID) {
                        runOnce = true;
                        telemetry.addLine("Starting PID control");
                        double angle = drivingLibrary.getIMUAngle();// get x value for center
                        telemetry.addData("starting angle", angle);
                        diff = (float) angle; // divide by 400 because otherwise turn x value will be too large
                        totalDiff += diff; // integral = sum of all previous error
                        float tval = diff*pidVals[0]+totalDiff*pidVals[1]+(lastDiff-diff)*pidVals[2];
                        if (Math.abs(diff) >= .1) {
                            drivingLibrary.bevelDrive(0, 0, tval);
                        }
                        lastDiff = diff; // update last difference
                        telemetry.addData("Diff", diff);
                        telemetry.addData("Motor Powers", drivingLibrary.getMotorPower());
                        telemetry.addData("turn value", tval);
                        telemetry.update();
                        sleep(30000);

                        /**
                         * PID
                         * corrects error (in this case, how far the gold mineral center is from the camera center)
                         * Proportional: constant * error
                         * Integral: constant * sum of total error so far
                         * Derivative: constant * rate of change of error
                         */
                    }


                }


                telemetry.update();
            }

            //tfod.shutdown();
        }
    }

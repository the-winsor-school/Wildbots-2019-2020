package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.libraries.DrivingLibrary;
import org.firstinspires.ftc.teamcode.BarryBot;


@TeleOp(name = "EncoderTesting", group = "BarryBot")
//@Autonomous(name="Pushbot: Auto Drive By Encoder", group="Pushbot")
@Disabled

public class EncoderTesting extends LinearOpMode {

    // declaring OpMode members
    BarryBot robot   = new BarryBot();   // Use a BarryBot hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    double servoPos = 0.3; //init servos


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, this);

        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition(),
                robot.leftRear.getCurrentPosition(),
                robot.rightRear.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {

            //change driving mode
            if (gamepad1.b) {
                robot.drivingMode++;
                robot.drivingMode %= DrivingMode.values().length;
                robot.drivingLibrary.setMode(robot.drivingMode);
            }
            //strafe

            if (gamepad1.y) {
                robot.drivingLibrary.setSpeed(.75);
            }

            robot.drivingLibrary.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("Status", "Running");
            telemetry.addData("Brake Mode", robot.drivingLibrary.getMode());

            // gamepad 2 up/down to move grab arm up/down
            if (gamepad2.dpad_up){
                //drives a foot backwards
                robot.grabArm.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                robot.grabArm.setPower(-1);
            }
            else {
                robot.grabArm.setPower(0);
            }

            // gamepad 2 left right to open/close the grab hand
            robot.grabHand.setPosition(servoPos);
            if (gamepad2.dpad_left) {
                servoPos = 1;
            }
            else if (gamepad2.dpad_right && servoPos > 0) {
                servoPos = 0.25;
            }
            else {
                //servoPos += .001;
            }

            telemetry.addData("Servo position: ", servoPos);

            // gamepad 2 a b to flip the drag arm

            //positive values go forwards
            if (gamepad2.a) {
                robot.dragArm.setPosition(1);
            }
            else if (gamepad2.b) {
                robot.dragArm.setPosition(-.5);
            }
            else {
                robot.dragArm.setPosition(0);
            }

            telemetry.update();

        }
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.rightFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.leftFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = robot.rightRear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.leftRear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.rightFront.setTargetPosition(newLeftTarget);
            robot.leftFront.setTargetPosition(newRightTarget);
            robot.rightRear.setTargetPosition(newLeftTarget);
            robot.leftRear.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.rightFront.setPower(Math.abs(speed));
            robot.leftFront.setPower(Math.abs(speed));
            robot.rightRear.setPower(Math.abs(speed));
            robot.leftRear.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightFront.isBusy() && robot.leftFront.isBusy() && robot.leftRear.isBusy() && robot.leftRear.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.rightFront.getCurrentPosition(),
                        robot.leftFront.getCurrentPosition(),
                        robot.leftRear.getCurrentPosition(),
                        robot.rightRear.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.rightFront.setPower(0);
            robot.rightRear.setPower(0);
            robot.leftFront.setPower(0);
            robot.leftRear.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}

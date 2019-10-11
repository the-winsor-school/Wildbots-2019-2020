package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.libraries.DrivingLibrary;

@Autonomous(name = "Right - Land and Push Center Mineral", group = "Finished")
public class AutonRight extends LinearOpMode {

    DrivingLibrary drivingLibrary;
    int drivingMode;

    // latch arm
    DcMotor latchArm;

    // intake arm
    CRServo intakeSpinServo;
    Servo intakeFlipServo;
    Servo intakeExtendArm;
    DcMotor intakeRotateArm;
    Servo drawerStopServo;

    public void runOpMode() throws InterruptedException {
        drivingLibrary = new DrivingLibrary(this);
        drivingLibrary.setSpeed(1);
        drivingMode = 0;
        drivingLibrary.setMode(drivingMode);

        // intake cr servo: rev hub 1 servo port 0
        intakeSpinServo = hardwareMap.get(CRServo.class, "intakeSpinServo");

        // intake flip servo: rev hub 1 servo port 1
        intakeFlipServo = hardwareMap.get(Servo.class, "intakeFlipServo");

        // intake arm extending winch servo: rev hub 1 servo port 2
        intakeExtendArm = hardwareMap.get(Servo.class, "intakeExtendArm");

        // intake arm rotational dc motor: rev hub 1 motor port 1
        intakeRotateArm = hardwareMap.get(DcMotor.class, "intakeRotateArm");

        // latch motor: rev hub 1 motor port 0
        latchArm = hardwareMap.get(DcMotor.class, "latchArm");

        // rev hub 1 port 3
        drawerStopServo = hardwareMap.get(Servo.class, "drawerStopServo");

        latchArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        boolean ranOnce = false;

        drawerStopServo.setPosition(0.9);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (!ranOnce){
                //land
                latchArm.setPower(.75);
                sleep(7000);
                latchArm.setPower(0);

                drivingLibrary.driveStraight(0.75f, 0);
                sleep(500);
                drivingLibrary.floatStop();

                //reset arm
                latchArm.setPower(-.75);
                sleep(5000);

                //drive backwards
                drivingLibrary.driveStraight(0, -.75f);
                sleep(3000);
                drivingLibrary.brakeStop();

                //drop of marker
                drawerStopServo.setPosition(0.39);
                intakeExtendArm.setPosition(.7);
                intakeFlipServo.setPosition(.55);

            }
            ranOnce = true;
        }
    }

    //land - extend arm, strafe, unextend arm

    //drive to depot, drop marker (possibly sample on the way)
}

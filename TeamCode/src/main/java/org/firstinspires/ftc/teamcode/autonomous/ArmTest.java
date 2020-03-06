package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Autonomous
public class ArmTest extends LinearOpMode {
    DcMotor grabArm;
    @Override
    public void runOpMode() throws InterruptedException {
        grabArm = hardwareMap.get(DcMotor.class, "grabArm");
        boolean ranOnce = false;
        waitForStart();

        if (opModeIsActive()) {
            if (!ranOnce) {
                for (float i = -.4f; i > -1; i-= .15) {
                    grabArm.setPower(i);
                    sleep(100);
                }
                grabArm.setPower(-1);
                sleep(1875);
                grabArm.setPower(0);
                ranOnce = true;
            }
        }
    }
}

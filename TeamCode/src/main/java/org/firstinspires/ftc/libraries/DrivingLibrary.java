package org.firstinspires.ftc.libraries;

/**
 * Created by lamanwyner on 12/29/17.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.enums.DrivingMode;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Arrays;

public class DrivingLibrary {
    // hardware variables
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private DcMotor[] allMotors;
    private HardwareMap hardwareMap;
    private double[] strafeBias;
    private double[] strafePowers;

    // sensor variables
    private BNO055IMU imu; //gyroscope in rev hub
    private Orientation angles;
    private Acceleration gravity;

    // other variables
    private double speedSetting; //sets max speed
    private DrivingMode drivingMode;
    private OpMode opMode;

    public DrivingLibrary(OpMode opMode) {
        /* library for functions related to drive train

         */
        this.opMode = opMode;
        hardwareMap = opMode.hardwareMap;

        leftFront = hardwareMap.tryGet(DcMotor.class, "leftFront");
        rightFront = hardwareMap.tryGet(DcMotor.class, "rightFront");
        leftRear = hardwareMap.tryGet(DcMotor.class, "leftRear");
        rightRear = hardwareMap.tryGet(DcMotor.class, "rightRear");

        allMotors = new DcMotor[] {leftFront, rightFront, leftRear, rightRear};
        for (DcMotor motor : allMotors) {
            if (motor == null) {
                opMode.telemetry.addLine("Wrong configuration");
                opMode.telemetry.update();
                opMode.stop();
            }
        }

        rightRear.setDirection(DcMotor.Direction.REVERSE); //motors face opposite directions so one side is reversed
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        //for imu set up
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        strafeBias = new double[] {1, 1, 1, 1};
    }

    //for incrementally changing strafe bias for testing
    public void updateStrafeBias(int wheel, int multiplier) {
        strafeBias[wheel] += (.01 * multiplier);
    }

    //displays strafe bias on phone
    public void printStrafeBias() {
        opMode.telemetry.addData("fl", strafeBias[0]);
        opMode.telemetry.addData("fr", strafeBias[1]);
        opMode.telemetry.addData("rr", strafeBias[2]);
        opMode.telemetry.addData("rl", strafeBias[3]);
    }

    //strafing on one joystick with twist on the other
    public void drive(float x, float y, float t) {
        double vd = strafeSpeed(x, y);
        double theta = Math.atan2(y, x);
        if (Math.abs(x) <= 0.05) { //dead zone start
            x = 0;
        }
        if (Math.abs(y) <= 0.05) {
            y = 0;
        }
        if (Math.abs(t) <= 0.05) {
            t = 0;
        } //dead zone end
        double vt = t;
        //in order -- lF, rF, rR, lR
        strafePowers = new double[] {vd * Math.sin(theta + Math.PI/4) - vt, 1.05 * vd * Math.sin(theta - Math.PI/4) + vt,
                vd * Math.sin(theta + Math.PI/4) + vt, vd * Math.sin(theta - Math.PI/4) - vt};//trying to correct some strafing bias

        strafeScale(strafePowers);

        leftFront.setPower(-strafePowers[0] * speedSetting);
        rightFront.setPower(-strafePowers[1] * speedSetting);
        rightRear.setPower(strafePowers[2] * speedSetting);
        leftRear.setPower(strafePowers[3] * speedSetting);
    }

    public double strafeSpeed(float x, float y) {
        double d = Math.sqrt(x*x + y*y);
        if (d > 1) {
            d = 1;
        }
        return d;
    }

    public double[] strafeScale(double[] strafePowers) {
        double maxPower = Math.abs(strafePowers[0]);
        for (int i = 1; i < 4; i++) {
            if (Math.abs(strafePowers[i]) > maxPower) {
                maxPower = Math.abs(strafePowers[i]);
            }
        }
        double scale = maxPower;
        if (scale >= 1) {
            for (int i = 0; i < 4; i++) {
                strafePowers[i] /= scale;
            }
        }
        return strafePowers;
    }


    /**
     *
     * @param x joystick.x
     * @param y joystick.y
     */
    public void strafe(float x, float y) {

        float maxSpeed = (float)Math.sqrt(x*x + y*y);
        //float maxSpeed = Math.max(y + x, y - x);
        double multiplier = 1;

        if (maxSpeed > 1) {
            multiplier = 1 / maxSpeed; //values normalised to be less than or equal to one
        }

        leftFront.setPower(-multiplier * speedSetting * (y + x) * strafeBias[0]);
        rightFront.setPower(multiplier * speedSetting * (y - x) * strafeBias[1]);
        rightRear.setPower(multiplier * speedSetting * (y + x) * strafeBias[2]);
        leftRear.setPower(-multiplier * speedSetting * (y - x) * strafeBias[3]);
    }

    public void turn(float x, float y) {
        float maxSpeed = Math.max(y + x, y - x);
        double multiplier = 1;

        if (maxSpeed > 1) {
            multiplier = 1 / maxSpeed;
        }

        leftFront.setPower((y + x) * speedSetting * multiplier * strafeBias[0]);
        leftRear.setPower((y + x) * speedSetting * multiplier * strafeBias[1]);
        rightFront.setPower((y - x) * speedSetting * multiplier * strafeBias[2]);
        rightRear.setPower((y - x) * speedSetting * multiplier * strafeBias[3]);
    }

    public void turn(double radians) {
        double k = 1;

        imu.startAccelerationIntegration(new Position(), new Velocity(), 500);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    public void setSpeed(double speed) {
        speedSetting = speed;
    }

    public void setMode(int i) {
        DrivingMode[] values = DrivingMode.values();
        drivingMode = values[i];

        switch (drivingMode) {
            case FLOAT_STOP:
                for (DcMotor motor : allMotors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } //coasts when it stops
                break;
            case BRAKE_STOP:
                for (DcMotor motor : allMotors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } //fully brakes
                break;
        }
    }

    public void setMode(DrivingMode d) {
        drivingMode = d;

        switch (drivingMode) {
            case FLOAT_STOP:
                for (DcMotor motor : allMotors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                break;
            case BRAKE_STOP:
                for (DcMotor motor : allMotors) {
                    motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
                break;
        }
    }

    public String getMode() {
        return drivingMode.getStringValue();
    }

    public void setMotorSpeed(double speed) {
        for (DcMotor motor : allMotors) {
            motor.setPower(speed);
        } //physically sets motor speed
    }

    public String getMotorPower() {
        double[] powers = new double[] {leftFront.getPower(), leftRear.getPower(), rightFront.getPower(), rightRear.getPower()};
        return Arrays.toString(powers);
    }

    public void stopDrivingMotors() {
        for (DcMotor motor : allMotors) {
            motor.setPower(0);
        }
    }

    public void floatStop() {
        for (DcMotor motor : allMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setPower(0);
        }
    }

    public void brakeStop() {
        for (DcMotor motor : allMotors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    private void resetEncoders() {
        for (DcMotor motor : allMotors) {
            if (motor != null) motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}

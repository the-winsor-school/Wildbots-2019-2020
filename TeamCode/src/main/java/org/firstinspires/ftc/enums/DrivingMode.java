package org.firstinspires.ftc.enums;

/**
 * Created by lamanwyner on 1/25/18.
 */

public enum DrivingMode {
    FLOAT_STOP("Float"),
    BRAKE_STOP("Brake");

    String stringValue;

    DrivingMode(String s) {
        stringValue = s;
    }

    public String getStringValue() {
        return stringValue;
    }
}

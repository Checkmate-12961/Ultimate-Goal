package org.firstinspires.ftc.teamcode.FPS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Measurement {
    public BNO055IMU revIMU;
    public HardwareMap hardwareMap;

    public Measurement(Hardware robot){
        hardwareMap = robot.hardwareMap;
        revIMU = robot.revIMU;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        revIMU = hardwareMap.get(BNO055IMU.class, "imu");
        revIMU.initialize(parameters);
    }
}

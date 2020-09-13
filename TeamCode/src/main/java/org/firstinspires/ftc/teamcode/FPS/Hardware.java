package org.firstinspires.ftc.teamcode.FPS;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 *  Hello! This is the hardware class. You do all of the initalization code for hardware here,
 *  making it a lot easier to create new opmodes. Better yet, if you change a motor or server
 *  here, it will change it in every class that uses it!
 *
 * */


public class Hardware {

    /**
     * As a part of this section, please remember:
     * If you need to create a variable of a new type, please remember to create a key for each variable!
     * It's vital that you can go back and remember what each variable does.
     */

    // DOUBLES:
    public double theta, forwardSpeed, finaltheta, robotSpeed, directionSpeed, leftfront, rightfront, leftback, rightback;
    /**
     * Please list what each double variable is for:
     * example: example is a variable used as an example for this list
     * varname: text
     * varname: text
     */


    // MOTORS:
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    /**
     * Please list what each servo does:
     * exampleservo: the example servo is used as an example, and is located on the rear of the robot
     * varname: text
     * varname: text
     */


    // SENSORS:
    public BNO055IMU revIMU;
    /**
     * Please list what each sensor is for:
     * examplesesor: the example sensor is used to detect bad memes, and is located on the right side of the robot
     * blockToggle: this is to detect if a block has been obtained and is in the collection bin
     * revIMU: this is the REV Expansion Hub's internal gyro/accelerometer, and is partially used to
     *      determine the location and orientation of the robot
     * varname: text
     * varname: text
     */


    // OTHER CLASSES:
    public Measurement sensorSuite;
    public HardwareMap hardwareMap; // Especially don't delete this one

    //INTAKE LEFT AND WINCH LEFT ARE X1 AND Y RESPECTIVELY

    public void map(HardwareMap map){
        hardwareMap = map; // this sets the hardwareMap variable in this class to the hardwareMap variable from inside the opmode

        /* Motors: */
        leftFront = hardwareMap.get(DcMotor.class, "LF");
        leftBack = hardwareMap.get(DcMotor.class, "LB");
        rightFront = hardwareMap.get(DcMotor.class, "RF");
        rightBack = hardwareMap.get(DcMotor.class, "RB");

        /* Sensors: */
        revIMU = hardwareMap.get(BNO055IMU.class, "imu");

        /* Set Motor Behavior: */
        // Drivetrain:
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders...
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // ...then turn them off.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        sensorSuite = new Measurement(this); // parses this hardware class into the Measurement class so it can use the robot's hardware
    }

    public void drivePowerCalculate(double xVectorLeft, double yVectorLeft, double xVectorRight, double yVectorRight){
        robotSpeed = Math.sqrt(Math.pow(xVectorLeft, 2) + Math.pow(xVectorLeft, 2));
        theta = Math.atan2(-xVectorLeft, yVectorLeft);
        if (yVectorRight == 0) {
            forwardSpeed = -yVectorLeft;
        } else {
            forwardSpeed = -yVectorRight;
        }
        finaltheta = -theta + (Math.PI/4);
        directionSpeed = -xVectorRight;

        leftfront = (robotSpeed * Math.sin(finaltheta) - directionSpeed) + forwardSpeed;
        leftback = (robotSpeed * Math.cos(finaltheta) - directionSpeed) + forwardSpeed;
        rightfront = (robotSpeed * Math.cos(finaltheta) + directionSpeed) + forwardSpeed;
        rightback = (robotSpeed * Math.sin(finaltheta)+ directionSpeed) + forwardSpeed;
    }

    public void setPower( double lf, double lb, double rf, double rb){
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }
}

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FPS.Hardware;



@TeleOp(name="Tele Relative", group="TeleOP")
public class RelativeOP extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final Hardware robot = new Hardware(); // Custom Class

    // Declare OpMode members.
    double lB, lF, rB, rF;

    // Declare vars for polar / rect stuff
    public double x, y, r, theta;
    
    // Rectangular coordinates to polar
    public void rectToPolar(double xIn, double yIn) {
        r     = Math.sqrt(xIn * xIn + yIn * yIn);
        theta = Math.atan2(yIn, xIn);
    }
    
    // Polar coordinates to rectangular
    public void polarToRect(double rIn, double thetaIn){
        x = Math.cos( thetaIn ) * rIn;
        y = Math.sin( thetaIn ) * rIn;
    }

    // Custom Method
    public void runDrivetrain() {


        /* We store the angle from the IMU's gyro both to prevent wonkiness if the angle changes partway through,
         * and also to make it easy to change which axis we read from, based on where the control hub / phone is
         * mounted.
         */
        double angle = robot.revIMU.getAngularOrientation().firstAngle;
        double radAngle;

        // This if statement is used to translate the janky way the IMU outputs angles into radians
        if (angle<0){
            radAngle = Math.toRadians(angle + 360);
        } else {
            radAngle = Math.toRadians(angle);
        }
        
        // Add the angle to the joystick input after converting to polar
        rectToPolar(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double finalTheta = theta + radAngle;

        // Sanity check
        while (finalTheta >= 2.0 * Math.PI) {
            finalTheta -= 2.0 * Math.PI;
        } while (finalTheta < 0){
            finalTheta += 2.0 * Math.PI;
        }
        
        // Convert it back to rectangular and calculate the drive power for each motor
        polarToRect(r, finalTheta);
        robot.drivePowerCalculate(x, y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        double gearSpeed = BaseOP.manageSpeed(.5, gamepad1.right_trigger, gamepad1.left_trigger);
        lF = gearSpeed * robot.leftfront;
        lB = gearSpeed * robot.leftback;
        rF = gearSpeed * robot.rightfront;
        rB = gearSpeed * robot.rightback;
    }


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        robot.map(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            runDrivetrain();

            robot.setPower(Range.clip(lF, -1, 1), Range.clip(lB, -1, 1), Range.clip(rF, -1, 1), Range.clip(rB, -1, 1));

            // Show the elapsed game time and gyroscope data.
            telemetry.addData("Status", "Run time: " + runtime.toString());
            telemetry.addData("Angle","First: " + robot.revIMU.getAngularOrientation().firstAngle);
            telemetry.addData("Angle","Second: " + robot.revIMU.getAngularOrientation().secondAngle);
            telemetry.addData("Angle","Third: " + robot.revIMU.getAngularOrientation().thirdAngle);
            telemetry.update();

        }
    }
}

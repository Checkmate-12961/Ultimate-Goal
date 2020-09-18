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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.FPS.Hardware;

@TeleOp(name="Tele Base", group="TeleOP")
public class BaseOP extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware(); // Custom Class

    // Declare OpMode members.
    double lB, lF, rB, rF;
    public static double manageSpeed(double base, double RT, double LT) {
        // Here we use the values from the left and right triggers to change the robot's speed
        double speed = base;
        // (.0 - speed) and (speed - .2) are there to allow for scaling with the base value.
        // As base shrinks, the impact of RT will increase, and the impact of LT will decrease.
        // The same works in reverse as base grows.
        speed += (.9 - speed) * RT;
        speed -= (speed - .2) * LT;
        return speed;
    }
    public void runDrivetrain() { // Custom Method

        robot.drivePowerCalculate(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.right_stick_y);

        // Here we calculate gearSpeed based off of the values of gamepad1's left_trigger and right_trigger (which are analog)
        double gearSpeed = manageSpeed(.5, gamepad1.right_trigger, gamepad1.left_trigger);
        // We then apply that speed to each motor
        lF = gearSpeed * robot.leftfront;
        lB = gearSpeed * robot.leftback;
        rF = gearSpeed * robot.rightfront;
        rB = gearSpeed * robot.rightback;
    }


    @Override
    public void runOpMode() {
        robot.map(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

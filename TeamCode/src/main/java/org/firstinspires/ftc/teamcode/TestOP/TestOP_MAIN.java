package org.firstinspires.ftc.teamcode.TestOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FPS.Hardware;

@TeleOp(name = "Test gamepad", group="TestOP")
public class TestOP_MAIN extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.map(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        while ( opModeIsActive() ) {
            int dpadY = 0;
            int dpadX = 0;
            if (gamepad1.dpad_up){
                dpadY = 1;
            } else if (gamepad1.dpad_down){
                dpadY = -1;
            }
            if (gamepad1.dpad_right){
                dpadX = 1;
            } else if (gamepad1.dpad_left){
                dpadX = -1;
            }

            // Show the elapsed game time and gamepad inputs.
            telemetry.addData("Buttons", "a: "+gamepad1.a+", b: "+gamepad1.b+", x: "+gamepad1.x+", y: "+gamepad1.y);
            telemetry.addData("Dpad", "UP: "+dpadY+", RT: "+dpadX);
            telemetry.addData("LStick X",gamepad1.left_stick_x);
            telemetry.addData("LSitick Y", gamepad1.left_stick_y);
            telemetry.addData("RStick X",gamepad1.right_stick_x);
            telemetry.addData("RStick Y",gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}

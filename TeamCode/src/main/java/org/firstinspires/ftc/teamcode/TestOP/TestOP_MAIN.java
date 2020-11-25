package org.firstinspires.ftc.teamcode.TestOP;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Test gamepad", group="TestOP")
public class TestOP_MAIN extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
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
            Vector2d lStickInput = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
            Pose2d lStickPlace = new Pose2d(
                    lStickInput.getX(),
                    lStickInput.getY(),
                    0
            );
            Vector2d rStickInput = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
            Pose2d rStickPlace = new Pose2d(
                    lStickInput.getX(),
                    lStickInput.getY(),
                    0
            );
            // Show the elapsed game time and gamepad inputs.
            telemetry.addData("Buttons", "a: "+gamepad1.a+", b: "+gamepad1.b+", x: "+gamepad1.x+", y: "+gamepad1.y);
            telemetry.addData("Dpad", "UP: "+dpadY+", RT: "+dpadX);
            telemetry.addData("LStick θ",lStickPlace.getHeading());
            telemetry.addData("RStick θ", rStickPlace.getHeading());
            telemetry.update();
        }
    }
}

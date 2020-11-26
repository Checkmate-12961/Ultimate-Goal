package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;

@Autonomous(group = "Meet start")
public class SetPosition extends LinearOpMode {
    Pose2d pose;
    @Override
    public void runOpMode() {
        waitForStart();
        telemetry.addData("Start","a: BL, b: BR, x: RL, y: RR");
        telemetry.update();
        boolean a = false;
        boolean b = false;
        boolean x = false;
        boolean y = false;
        while (!(a || b || x || y)) {
            a = gamepad1.a;
            b = gamepad1.b;
            x = gamepad1.x;
            y = gamepad1.y;
        }
        if (a){
            pose = new Pose2d(-70.5, 44.25 , Math.toRadians(180) );
        } else if (b){
            pose = new Pose2d(-70.5, 20.25 , Math.toRadians(180) );
        } else if (x){
            pose = new Pose2d(-70.5, -20.25 , Math.toRadians(180) );
        } else {
            pose = new Pose2d(-70.5, -44.25 , Math.toRadians(180) );
        }
        PoseStorage.currentPose = pose;
    }
}

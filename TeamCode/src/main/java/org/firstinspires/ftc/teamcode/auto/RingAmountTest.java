/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class RingAmountTest extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webCam;
    Vision.RingDeterminationPipeline pipeline;
    private static Vision.RingDeterminationPipeline.RingPosition position = Vision.RingDeterminationPipeline.RingPosition.FOUR;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.setAutoClear(false);
        Telemetry.Item initItem = telemetry.addData("Initializing...","Setting up hardware");
        telemetry.update();

        initItem.setValue("Starting camera feed");
        telemetry.update();
        // Camera stuff
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camra"), cameraMonitorViewId);
        pipeline = new Vision.RingDeterminationPipeline();
        webCam.setPipeline(pipeline);

        //listens for when the camera is opened
        webCam.openCameraDeviceAsync(() -> {
            //if the camera is open start steaming
            webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT  );
        });

        initItem.setValue("Checking ring position");
        telemetry.update();

        int ringPos = pipeline.getAnalysis();
        Telemetry.Item ringPosItem = telemetry.addData("RingPosGuess",ringPos);
        telemetry.addData("RingAmountGuess", position);



        initItem.setValue(String.format("Done. Took %f milliseconds",runtime.milliseconds()));
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;
        telemetry.removeItem(initItem);
        double initTime = runtime.milliseconds();

        Telemetry.Item runtimeItem = telemetry.addData(
                "Runtime",
                String.format(
                        "%fms",
                        runtime.milliseconds()-initTime
                ));

        while (opModeIsActive() && !isStopRequested()) {
            runtimeItem.setValue(
                    String.format(
                            "%fms",
                            runtime.milliseconds()-initTime
                    ));
            ringPos = pipeline.getAnalysis();
            ringPosItem.setValue(ringPos);
            telemetry.update();

        }
    }
}

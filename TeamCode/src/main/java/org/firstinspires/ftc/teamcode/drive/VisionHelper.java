package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionHelper{/*
    public static int TopLeftX = 210;
    public static int TopLeftY = 170;
    public static int Width = 90;
    public static int Height = 60;
    public static int FourRingThresh = 140;
    public static int OneRingThresh = 132;

    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }

    public static class RingDeterminationPipeline extends OpenCvPipeline
    {
        //Some color constants
        private static final Scalar BLUE = new Scalar(0, 0, 255);
        private static final Scalar GREEN = new Scalar(0, 255, 0);

        //The core values which define the location and size of the sample regions
        private final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(TopLeftX,TopLeftY);

        private final Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        private final Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + Width,
                REGION1_TOPLEFT_ANCHOR_POINT.y + Height);

        /*
         * Working variables
         *//*
        private Mat region1_Cb;
        private final Mat YCrCb = new Mat();
        private final Mat Cb = new Mat();
        private int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         *//*
        private void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FourRingThresh){
                position = RingPosition.FOUR;
            }else if (avg1 > OneRingThresh){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
        public RingPosition getPosition(){
            return position;
        }
    }*/
}

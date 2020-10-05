/*
This program was written by the FTC KTM #12529 team at the Polytechnic University in 2020.

This class is needed to process data from the camera.
This code was taken from https://www.youtube.com/watch?v=-QFoOCoaW7I

Our team wishes you all the best for the upcoming tournament.
All versions of the code starting from 2020 you can see here: https://github.com/RT-four/FTC-KTM-12529-TeamCode

Directed by RT-4(Philipp Vasiliev) and Dafter(Daniil Simonovsky)
*/
package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class EasyOpenCVVision extends OpenCvPipeline {
    // This enum contains the possible number of rings
    public enum RingPosition {
        FOUR,
        ONE,
        NONE
    }

    // Color constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    // Constants for determining the number of rings based on the red content
    final int FOUR_RING_THRESHOLD = 150;
    final int ONE_RING_THRESHOLD = 135;

    // Upper-left point of the rectangle where the rings will be defined
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
    // The width of the rectangle where the rings will be defined
    static final int REGION_WIDTH = 35;
    // The height of the rectangle where the rings will be defined
    static final int REGION_HEIGHT = 25;

    // Creating the upper-left point of the rectangle where the rings will be defined
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    // Creating the lower-right point of the rectangle where the rings will be defined
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Working variables
    Mat region1_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1; // The amount of red in the specified rectangle

    // Variable where the number of rings will be stored at the moment
    public volatile RingPosition position = RingPosition.FOUR;

    // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];

        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        position = RingPosition.FOUR; // Record our analysis
        if (avg1 > FOUR_RING_THRESHOLD) {
            position = RingPosition.FOUR;
        } else if (avg1 > ONE_RING_THRESHOLD) {
            position = RingPosition.ONE;
        } else {
            position = RingPosition.NONE;
        }

//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                GREEN, // The color the rectangle is drawn in
//                -1); // Negative thickness means solid fill

        return input;
    }

    public int getAnalysis() {
        return avg1;
    }
}



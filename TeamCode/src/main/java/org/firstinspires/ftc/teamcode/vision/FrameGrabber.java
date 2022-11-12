package org.firstinspires.ftc.teamcode.vision;


import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.vision.filters.ColorFilter;
import org.firstinspires.ftc.teamcode.vision.filters.Filter;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class FrameGrabber extends OpenCVPipeline {
    public Filter colorFiler = new ColorFilter();

    public double leftGuide = 160;
    public double rightGuide = 320;
    public double mask = 0;
    public double threshold = 70;
    public String position = "";

    public double perfectRatio = 1;
    public double areaWeight = 0.01; // Since we're dealing with 100's of pixels
    public double minArea = 1000;
    public double ratioWeight = 100; // Since most of the time the area diffrence is a decimal place
    Point p1 = new Point(0, mask);
    Point p4 = new Point(960, 1280);
    private Mat workingMat = new Mat();
    private Mat displayMat = new Mat();
    private Mat yellowMask = new Mat();
    private Mat hiarchy = new Mat();
    private boolean firstIteration = true;

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // Copy input mat to working/display mats
        rgba.copyTo(displayMat);
        rgba.copyTo(workingMat);
        rgba.release();

        if (firstIteration) {
            mask = displayMat.height() / 2;
        }

        Imgproc.rectangle(workingMat, new Point(0, mask), new Point(displayMat.width(), 0), new Scalar(255, 255, 255), -1, 8, 0);
        Imgproc.rectangle(displayMat, new Point(0, mask), new Point(displayMat.width(), 0), new Scalar(255, 255, 255), -1, 8, 0);

        // Generate Masks
        colorFiler.processFrame(workingMat.clone(), yellowMask, mask, threshold);

        // Blur and find the countours in the masks
        List<MatOfPoint> contoursYellow = new ArrayList<>();

        Imgproc.blur(yellowMask, yellowMask, new Size(4, 4));

        // Mask of the section thats useless and just making havishs life harder.
        Imgproc.line(displayMat, new Point(0, mask), new Point(640, mask), new Scalar(255, 0, 0));

        Imgproc.findContours(yellowMask, contoursYellow, hiarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(displayMat, contoursYellow, -1, new Scalar(230, 70, 70), 2);

        // Prepare to find best yellow (gold) results
        Rect chosenYellowRect = null;
        double chosenYellowScore = Integer.MAX_VALUE;

        MatOfPoint2f approxCurve = new MatOfPoint2f();

        for (MatOfPoint cont : contoursYellow) {
            // Get bounding rect of contour
            Rect rect = Imgproc.boundingRect(cont);

            //Imgproc.line(input, center, center, new Scalar(0, 0, 0),5);

            double area = Imgproc.contourArea(cont);
            double areaDiffrence = 0.0;

            areaDiffrence = -area * areaWeight;


            // Just declaring vars to make my life eassy
            double x = rect.x;
            double y = rect.y;
            double w = rect.width;
            double h = rect.height;
            Point centerPoint = new Point(x + (w / 2), y + (h / 2));


            double cubeRatio = Math.max(Math.abs(h / w), Math.abs(w / h)); // Get the ratio. We use max in case h and w get swapped??? it happens when u account for rotation
            double ratioDiffrence = Math.abs(cubeRatio - perfectRatio);


            double finalDiffrence = (ratioDiffrence * ratioWeight) + (areaDiffrence * areaWeight);

            if (area > minArea) {
                chosenYellowScore = finalDiffrence;
                chosenYellowRect = rect;
            }
        }

        if (chosenYellowRect != null) {
            Imgproc.rectangle(displayMat,
                    new Point(chosenYellowRect.x, chosenYellowRect.y),
                    new Point(chosenYellowRect.x + chosenYellowRect.width, chosenYellowRect.y + chosenYellowRect.height),
                    new Scalar(255, 0, 0), 2);

            Imgproc.putText(displayMat,
                    "Gold: " + String.format("%.2f X=%.2f", chosenYellowScore, (double) chosenYellowRect.x),
                    new Point(chosenYellowRect.x - 5, chosenYellowRect.y - 10),
                    Core.FONT_HERSHEY_PLAIN,
                    1.3,
                    new Scalar(0, 255, 255),
                    2);
            Imgproc.putText(displayMat,
                    calcAllign(chosenYellowRect.x),
                    new Point(100, 100),
                    Core.FONT_HERSHEY_PLAIN,
                    5,
                    new Scalar(0, 255, 255),
                    2);
        } else {
            Imgproc.putText(displayMat,
                    calcAllignRight(),
                    new Point(100, 100),
                    Core.FONT_HERSHEY_PLAIN,
                    5,
                    new Scalar(0, 255, 255),
                    2);
        }

        Imgproc.line(displayMat, new Point(leftGuide, 0), new Point(leftGuide, 1280), new Scalar(255, 0, 0));

        firstIteration = false;
        return displayMat;
    }

    /**
     * @return 0 if left, 1 if middle, 2 if right, 4 if uh oh
     */
    public String calcAllign(double x) {
        if (x < leftGuide) {
            position = "LEFT";
            return "LEFT";
        } else if (x > leftGuide) {
            position = "MIDDLE";
            return "MIDDLE";
        } else /*if (x > rightGuide)*/ {
            position = "RIGHT";
            return "RIGHT";
        } /*else {
            position = "oh cmon havish...";
            //paranoia
            return "NANI";
        }*/
    }

    public String calcAllignRight() {
        position = "RIGHT";
        return "RIGHT";
    }

}
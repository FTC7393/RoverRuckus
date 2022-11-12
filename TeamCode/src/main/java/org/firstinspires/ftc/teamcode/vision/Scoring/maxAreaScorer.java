package org.firstinspires.ftc.teamcode.vision.Scoring;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class maxAreaScorer extends Scorer {
    public double perfectArea = 1.0; //TODO: Find actuall val

    public maxAreaScorer(double perfectArea) {
        this.perfectArea = perfectArea;
    }


    @Override
    public double calculateDifference(MatOfPoint contours) {
        double score = Double.MAX_VALUE;
        MatOfPoint2f approxCurve = new MatOfPoint2f();
        MatOfPoint2f contour2f = new MatOfPoint2f(contours.toArray());

        //Processing on mMOP2f1 which is in type MatOfPoint2f
        double approxDistance = Imgproc.arcLength(contour2f, true) * 0.02;
        Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

        //Convert back to MatOfPoint
        MatOfPoint points = new MatOfPoint(approxCurve.toArray());

        // Get bounding rect of contour
        Rect rect = Imgproc.boundingRect(points);
        double x = rect.x;
        double y = rect.y;
        double w = rect.width;
        double h = rect.height;

        double area = w * h;

        // Score calculations
        double diffrence = Math.abs(perfectArea - area);


        return area;
    }
}

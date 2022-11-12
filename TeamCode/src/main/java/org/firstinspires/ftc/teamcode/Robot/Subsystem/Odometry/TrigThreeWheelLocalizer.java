package org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.List;


/*
 *    This class takes in 3 tracking encoder wheel positions and and outputs the pose of the robot
 *    in the field coordinates with the origin being where the robot started.
 *
 *
 *    ^
 *    |
 *    |
 *    | +X
 *    |
 *    |     +Y
 *    ---------------->  +90 degrees
 *
 *   Going forward increases the x value and going to the right increases the y value
 *   turning right increases the theta.
 */
public abstract class TrigThreeWheelLocalizer {
    private double chassisWidth = 38.5; //cm
    public static double dTheta = 0;
    private Pose2d poseEstimate;
    private List<Double> lastWheelPositions;
    double x = -143, y = 143, theta = Math.toRadians(-45);

    Telemetry telemetry;

    public TrigThreeWheelLocalizer(Telemetry telemetry) {
        this.telemetry = telemetry;

        lastWheelPositions = Collections.emptyList();
        //poseEstimate = new Pose2d(143, 143, Math.toRadians(135));
        poseEstimate = new Pose2d(-143, 143, Math.toRadians(-45));

    }

    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        if (!lastWheelPositions.isEmpty()) {

            double dL = wheelPositions.get(0) - lastWheelPositions.get(0);
            double dR = wheelPositions.get(1) - lastWheelPositions.get(1);
            double dM = wheelPositions.get(2) - lastWheelPositions.get(2);

            double dS = (dR + dL) / 2.0;
            dTheta = ((dR - dL) / chassisWidth);

            double avgTheta = theta + dTheta / 2.0;

            double dX = dS * Math.cos(avgTheta) + dM * Math.sin(avgTheta);
            double dY = dS * Math.sin(avgTheta) - dM * Math.cos(avgTheta);

            // Update current robot position.
            x += dX;
            y += dY;
            theta += dTheta;

            poseEstimate = new Pose2d(x, y, theta);
            lastWheelPositions = wheelPositions;

        }
        lastWheelPositions = wheelPositions;
    }

    public Pose2d getPoseEstimate() {
        double degrees = /*3.1415 / 2 +*/ poseEstimate.getHeading() * (180 / Math.PI);
        Pose2d poseEstimateCorrected = new Pose2d(
                poseEstimate.getX(),
                poseEstimate.getY(),
                degrees
        );

        return poseEstimateCorrected;
    }

    public abstract List<Double> getWheelPositions();
}

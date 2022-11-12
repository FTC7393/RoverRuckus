package org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry;

import java.util.Arrays;

public class PurePursuit {
    public double[][] origPath;
    public double[] distanceBetweenPoints;
    public static final double LOOK_AHEAD_DISTANCE = 30;
    public static final double MAX_VELOCITY = 121.92; // cm per second

    public PurePursuit(double[][] path) {
        this.origPath = doubleArrayCopy(path);
    }

    public static Vector2d intersect(double m1, double b1, double m2, double b2) {
        double x = (b1 - b2) / (m2 - m1);
        double y = m2 * x + b2;

        return new Vector2d(x, y);
    }

    public static double[] generateLine(Vector2d a, Vector2d b) {
        double m = (b.getY() - b.getX()) / (a.getY() - a.getX());

        double bb = b.getX() - (m * a.getX());

        double[] test = {m, bb};

        return test;
    }

    public static double[][] reverse(double[][] my_array) {
        int my_rows = my_array.length;
        int my_cols = my_array[0].length;
        double array[][] = new double[my_rows][my_cols];
        for (int i = my_rows - 1; i >= 0; i--) {
            array[my_rows - 1 - i][0] = my_array[i][0];
            array[my_rows - 1 - i][1] = my_array[i][1];
        }
        return array;
    }

    public static double[][] doubleArrayCopy(double[][] arr) {

        //size first dimension of array
        double[][] temp = new double[arr.length][arr[0].length];

        for (int i = 0; i < arr.length; i++) {
            //Resize second dimension of array
            temp[i] = new double[arr[i].length];

            //Copy Contents
            for (int j = 0; j < arr[i].length; j++)
                temp[i][j] = arr[i][j];
        }

        return temp;

    }

    public static double[][] inject(double[][] orig, int numToInject) {
        double morePoints[][];

        //create extended 2 Dimensional array to hold additional points
        morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];

        int index = 0;

        //loop through original array
        for (int i = 0; i < orig.length - 1; i++) {
            //copy first
            morePoints[index][0] = orig[i][0];
            morePoints[index][1] = orig[i][1];
            index++;

            for (int j = 1; j < numToInject + 1; j++) {
                //calculate intermediate x points between j and j+1 original points
                morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];

                //calculate intermediate y points  between j and j+1 original points
                morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];

                index++;
            }
        }

        //copy last
        morePoints[index][0] = orig[orig.length - 1][0];
        morePoints[index][1] = orig[orig.length - 1][1];
        index++;

        return morePoints;
    }

    public static double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {

        //copy array
        double[][] newPath = doubleArrayCopy(path);

        double change = tolerance;
        while (change >= tolerance) {
            change = 0.0;
            for (int i = 1; i < path.length - 1; i++)
                for (int j = 0; j < path[i].length; j++) {
                    double aux = newPath[i][j];
                    newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
                    change += Math.abs(aux - newPath[i][j]);
                }
        }

        return newPath;

    }

    public static double[] distanceBetweenPoints(double[][] path) {
        double[] dist = new double[path.length];
        dist[0] = 0;

        for (int i = 1; i < path.length; i++) {
            double distBetweenPoint = Math.sqrt(Math.pow((path[i][0] - path[i - 1][0]), 2) + Math.pow((path[i][1] - path[i - 1][1]), 2));
            dist[i] = dist[i - 1] + distBetweenPoint;
        }

        return dist;
    }

    public static int closestPoint(double[][] path, Pose2d curr) {
        double closestPrevious = Double.POSITIVE_INFINITY;
        int index = 0;
        for (int i = 1; i < path.length; i++) {
            double distBetweenPoint = Math.sqrt(Math.pow(path[i][0] - curr.getX(), 2) + Math.pow(path[i][1] - curr.getY(), 2));
            if (distBetweenPoint < closestPrevious) {
                closestPrevious = distBetweenPoint;
                index = i;
            }
        }
        return index;
    }

    /**
     * @param center center of circle (robot location)
     * @param r      radius of the circle (lookahead distance}
     * @return lookahead point
     */
    public static Vector2d getLookAheadPoint(double[][] path, Pose2d center, double r) {

        double[][] trimmed = Arrays.copyOfRange(path, 0, path.length - 1);
        double[][] reversed = reverse(trimmed);

        Vector2d a1 = new Vector2d(path[closestPoint(path, center)][0], path[closestPoint(path, center)][1]);
        Vector2d a2 = new Vector2d(path[closestPoint(path, center) + 1][0], path[closestPoint(path, center) + 1][1]);

        double[] pathLine = generateLine(a1, a2);
        // System.out.println(Arrays.toString(pathLine));

        double m = -(1 / pathLine[0]);
        // double b = -(m * center.getX());

        double bb = (m * center.getX() - center.getY()) / -1;

        Vector2d intersection = intersect(pathLine[0], pathLine[1], m, bb);
        System.out.println("Path that im near " + a1 + " " + a2);

        Vector2d closest = new Vector2d(path[closestPoint(path, center)][0], path[closestPoint(path, center) + 1][1]);
        System.out.println("closest: " + closest);

        int i = 0;
        for (double[] p : reversed) {
            int i_ = path.length - 2 - i;

            Vector2d vCenter = new Vector2d(closest.getX(), closest.getY());

            Vector2d e = new Vector2d(p[0], p[1]);
            Vector2d l = new Vector2d(path[i_ + 1][0], path[i_ + 1][1]);

            Vector2d d = l.minus(e); //Direction of vector of ray, from start to end
            Vector2d f = e.minus(vCenter); // Vector from center circle to ray start

            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - r * r;
            double discriminant = b * b - 4 * a * c;

            if (discriminant >= 0) {
                discriminant = Math.sqrt(discriminant);
                double t1 = (-b + discriminant) / (2 * a);
                double t2 = (-b - discriminant) / (2 * a);

                if (t1 >= 0 && t1 <= 1) {
                    return new Vector2d(p[0] + t1 * d.getX(), p[1] + t1 * d.getY());
                }
                if (t2 >= 0 && t2 <= 1) {
                    return e.plus(d.times(t2));

                }
            }
            i++;
        }
        return new Vector2d(Double.NaN, Double.NaN);
    }

    /**
     * + right
     * - left
     *
     * @param robot
     * @param l     lookaahead point
     * @return
     */
    public static Vector2d curvature(Pose2d robot, Vector2d l) {
        Vector2d robotVector = new Vector2d(robot.getX(), robot.getY());
        Vector2d vector = l.minus(robotVector);

        /*double angle = Math.toRadians(robot.getHeading());
        double side = Math.signum(Math.sin(angle) * (l.getX() - robot.getX()) - Math.cos(angle) * (l.getY() - robot.getY()));

        double a = -Math.tan(angle);
        double b = 1;
        double c = Math.tan(angle) * robot.getX() - robot.getY();

        double x = Math.abs(a * l.getX() + l.getY() + c) / Math.sqrt((a * a) + (b * b));

        //double L = Math.sqrt(Math.pow(l.getX(), 2) + Math.pow(l.getY(), 2));
        double curvature = (2 * x) / (LOOK_AHEAD_DISTANCE * LOOK_AHEAD_DISTANCE);

        double signedCurvature = curvature * side;*/

        return vector;
    }

    public static Vector2d getWheelVelocities(Vector2d c, double v) {
        // V = target robot velocity
        // L = target left wheel’s speed
        // R = target right wheel’s speed
        // C = curvature of arc
        // W = angular velocity of robot
        // T = track width

        double errorAt100 = 60;

        double p = 1 / errorAt100;

        //final double t = 20.64;

        //double kv = 1.0 / MAX_VELOCITY;
        //double FF = kv * 60.96;

        return new Vector2d(c.getX() * p, c.getY() * p);
    }

    public static Vector2d loop(double[][] path, double x, double y, double c, double lookahead) {
        Pose2d asd = new Pose2d(x, y, c);
        Vector2d l = getLookAheadPoint(path, asd, lookahead);

        Vector2d curvature = curvature(asd, l);

        return getWheelVelocities(curvature, 40);
    }

    public static void main(String[] args) {
        // NON-LOOP
        /*double[][] path = {
                {0, 0},
                {-143, 143},
                {-122, 122},
                {-21, 174},
                {-21, 210}
        };


        path = inject(path, 50);

        Pose2d curr = new Pose2d(-140, 130, -45);

        // LOOP
        System.out.println(getLookAheadPoint(path, curr, 30));
        System.out.println(PurePursuit.loop(path, curr.getX(), curr.getY(), curr.getHeading(), 30));*/

        double goal = -80;
        double curr = -81;
        double power = 1;

        double powerC;
        if (Math.abs(goal - curr) < Math.abs(360 - goal + curr)) {
            if ((goal - curr < 0)) {
                powerC = Math.abs(power);
            } else {
                powerC = -Math.abs(power);
            }
        } else {
            if (360 - goal + curr > 0) {
                powerC = Math.abs(power);
            } else {
                powerC = -Math.abs(power);
            }
        }

        System.out.println(Math.abs(((goal - curr) * 1 / 30)) * powerC);
    }
}

package org.firstinspires.ftc.teamcode.Robot.Subsystem;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.TrigThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.Robot.Util.PIDController;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

public class MecanumDrive extends TrigThreeWheelLocalizer {
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx backLeft;
    public static DcMotorEx backRight;
    public ElapsedTime eTime;
    LinearOpMode opMode;
    PIDController
            pidControllerX, pidControllerY, pidControllerC;

    Telemetry telemetry;
    private File file;
    private PrintStream logFile;
    private long delay;
    double errorAt100 = 60;

    public MecanumDrive(Robot robot, HardwareMap map, Telemetry telemetry, LinearOpMode opMode) {
        super(telemetry);
        this.opMode = opMode;
        this.telemetry = telemetry;

        frontLeft = map.get(DcMotorEx.class, "FL");
        frontRight = map.get(DcMotorEx.class, "FR");
        backLeft = map.get(DcMotorEx.class, "BL");
        backRight = map.get(DcMotorEx.class, "BR");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidControllerX = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerY = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerC = new PIDController(1 / 30.0, 0.0, 0.0);

        // Logging code
        file = new File(Environment.getExternalStorageDirectory(), createUniqueFileName("mecanum_drive", ".csv") + "");

        try {
            logFile = new PrintStream(new FileOutputStream(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        logFile.println("x,y,c");
        logFile.close();


        eTime = new ElapsedTime();
    }

    // ODOMETRY CODE ------------------------------------------------------------------------------

    /* Gets unaltered encoder position */
    public double getRawLeftPosition() {
        return frontRight.getCurrentPosition();
    }

    public double getRawCenterPosition() {
        return backRight.getCurrentPosition();
    }

    public double getRawRightPosition() {
        return -backLeft.getCurrentPosition();
    }

    /* Gets position in centimeters */
    public double getLeftPosition() {
        return ((Math.PI * 60.54) / 1440 * frontRight.getCurrentPosition()) / 10;
    }

    public double getCenterPosition() {
        return (((Math.PI * 60.54) / 1440) * backRight.getCurrentPosition()) / 10;
    }

    public double getRightPosition() {
        return -((Math.PI * 60.54) / 1440 * backLeft.getCurrentPosition()) / 10;
    }

    public Pose2d getPosition() {
        return getPoseEstimate();
    }

    // LINE FOLLOWING CODE -----------------------------------------------------------------------

    /**
     * Moves to a point using PID and odometry. The function also logs the position of the robot
     * into a CSV file so we can make cool graphs and plots of the robots position
     *
     * @param goal    Pose2d goal (x,y,c)
     * @param power   double power scalar
     * @param opMode1 Linear OpMode opMode so we can use methods such as opModeisActive()
     */
    public void lineTo(Pose2d goal, double power, LinearOpMode opMode1) throws InterruptedException {
        // Set PID coefficients
        pidControllerX = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerY = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerC = new PIDController(1 / errorAt100, 0.0, 0.05); //55.0

        double rDeadband = 5; //5cm

        // Calculate how far away we are from the goal
        double currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));
        double powerC;
        // Check wether opMode is running

        while ((opMode.opModeIsActive()) && (rDeadband < currentR || Math.abs(goal.getHeading() - getPosition().getHeading()) > 2)) {
            telemetry.addData("f", opMode1.gamepad1.left_stick_x);

            if (!opMode.opModeIsActive()) return;
            if (Math.abs(goal.getHeading() - getPosition().getHeading()) < Math.abs(360 - goal.getHeading() + getPosition().getHeading())) {
                if ((goal.getHeading() - getPosition().getHeading() < 0)) {
                    powerC = Math.abs(power);
                } else {
                    powerC = -Math.abs(power);
                }
            } else {
                if (360 - goal.getHeading() + getPosition().getHeading() > 0) {
                    powerC = Math.abs(power);
                } else {
                    powerC = -Math.abs(power);
                }
            }
            // Position logging in a CSV file
            //String CSV = getPosition().getX() + "," + getPosition().getY() + "," + getPosition().getHeading();
            //logFile.println(CSV);

            // Regenerate the current R
            currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

            // Use PID to find the X Y and C powers
            double xVal = pidControllerX.getError(getPosition().getX(), goal.getX(), 0.5);
            double yVal = pidControllerY.getError(getPosition().getY(), goal.getY(), 2.0);
            double cVal = pidControllerC.getError(getPosition().getHeading(), goal.getHeading(), 2.0);

            // Apply the powers
            fieldOriented(-xVal * 2, -yVal, Math.abs(cVal) * powerC, power, getPosition().getHeading());

            // Update position
            telemetry.update();
            update();
        }
        moveBy(0, 0, 0, 0);
    }

    /**
     * lineto but with a custom rDeadband value so if the robot is close it will stop.
     *
     * @param goal      Pose2d goal (x,y,c)
     * @param power     double power scalar
     * @param rDeadband deadband of how close the robot has to be before it stops
     * @param opMode1   Linear OpMode opMode so we can use methods such as opModeisActive()
     * @throws InterruptedException
     */
    public void lineTo(Pose2d goal, double power, double rDeadband, LinearOpMode opMode1) throws InterruptedException {
        // Set PID coefficients
        pidControllerX = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerY = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerC = new PIDController(1 / errorAt100, 0.0, 0.05); //55.0
        double powerC;
        ; //5cm

        // Calculate how far away we are from the goal
        double currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

        // Check wether opMode is running
        if (opMode.isStopRequested()) {
            return;
        } else {
            while (opMode.opModeIsActive() && (rDeadband < currentR || Math.abs(goal.getHeading() - getPosition().getHeading()) > 10)) {
                if (opMode.isStopRequested()) {
                    break;
                }
                if (Math.abs(goal.getHeading() - getPosition().getHeading()) < Math.abs(360 - goal.getHeading() + getPosition().getHeading())) {
                    if ((goal.getHeading() - getPosition().getHeading() < 0)) {
                        powerC = Math.abs(power);
                    } else {
                        powerC = -Math.abs(power);
                    }
                } else {
                    if (360 - goal.getHeading() + getPosition().getHeading() > 0) {
                        powerC = Math.abs(power);
                    } else {
                        powerC = -Math.abs(power);
                    }
                }
                // Position logging in a CSV file
                //String CSV = getPosition().getX() + "," + getPosition().getY() + "," + getPosition().getHeading();
                //logFile.println(CSV);

                // Regenerate the current R
                currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

                // Use PID to find the X Y and C powers
                double xVal = pidControllerX.getError(getPosition().getX(), goal.getX(), 0.5);
                double yVal = pidControllerY.getError(getPosition().getY(), goal.getY(), 2.0);
                double cVal = pidControllerC.getError(getPosition().getHeading(), goal.getHeading(), 2.0);

                telemetry.addData("Pos", getPosition());
                telemetry.addData("yVal", currentR);
                telemetry.addData("xVal", xVal);

                // Apply the powers
                fieldOriented(-xVal * 2, -yVal, Math.abs(cVal) * powerC, power, getPosition().getHeading());

                // Update position
                telemetry.update();
                update();
            }
            moveBy(0, 0, 0, 0);
        }
    }


    /**
     * Line to with a custom timeout value so if the robot gets stuck it wont break
     *
     * @param goal      Pose2d goal (x,y,c)
     * @param power     double power scalar
     * @param rDeadband deadband of how close the robot has to be before it stops
     * @param time      timeout value of how long the robot can atempt the movment befre t stops
     * @param opMode1   Linear OpMode opMode so we can use methods such as opModeisActive()
     */
    public void lineToTime(Pose2d goal, double power, double rDeadband, double time, LinearOpMode opMode1) throws InterruptedException {
        // Set PID coefficients
        pidControllerX = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerY = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerC = new PIDController(1 / errorAt100, 0.0, 0.05); //55.0

        double powerC = 1;

        ; //5cm
        eTime.reset();

        // Calculate how far away we are from the goal
        double currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

        // Check wether opMode is running
        if (opMode.isStopRequested()) {
            return;
        } else {
            while (opMode.opModeIsActive() && (eTime.time() < time && rDeadband < currentR || Math.abs(goal.getHeading() - getPosition().getHeading()) > 10)) {
                if (opMode.isStopRequested()) {
                    break;
                }
                if (Math.abs(goal.getHeading() - getPosition().getHeading()) < Math.abs(360 - goal.getHeading() + getPosition().getHeading())) {
                    if ((goal.getHeading() - getPosition().getHeading() < 0)) {
                        powerC = Math.abs(power);
                    } else {
                        powerC = -Math.abs(power);
                    }
                } else {
                    if (360 - goal.getHeading() + getPosition().getHeading() > 0) {
                        powerC = Math.abs(power);
                    } else {
                        powerC = -Math.abs(power);
                    }
                }

                // Position logging in a CSV file
                //String CSV = getPosition().getX() + "," + getPosition().getY() + "," + getPosition().getHeading();
                //logFile.println(CSV);

                // Regenerate the current R
                currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

                // Use PID to find the X Y and C powers
                double xVal = pidControllerX.getError(getPosition().getX(), goal.getX(), 0.5);
                double yVal = pidControllerY.getError(getPosition().getY(), goal.getY(), 2.0);
                double cVal = pidControllerC.getError(getPosition().getHeading(), goal.getHeading(), 2.0);

                telemetry.addData("cVal", cVal);
                telemetry.addData("heading", getPosition().getHeading());
                telemetry.addData("goal", goal.getHeading());
                // Apply the powers
                fieldOriented(-xVal * 2, -yVal, Math.abs(cVal) * powerC, power, getPosition().getHeading());

                // Update position
                telemetry.update();
                update();
            }
            moveBy(0, 0, 0, 0);
        }
    }


    /**
     * Arcs to a point using PID and odometry. The function also logs the position of the robot
     * into a CSV file so we can make cool graphs and plots of the robots position
     *
     * @param goal   Pose2d goal (x,y,c)
     * @param power  double power scalar
     * @param opMode Linear OpMode opMode so we can use methods such as opModeisActive()
     */
    public void arc(Pose2d goal, double power, LinearOpMode opMode) {
        // Set PID coefficients
        pidControllerX = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerY = new PIDController(1 / errorAt100, 0.0, 0.0);
        pidControllerC = new PIDController(1 / 25.0, 0.0, 0.0);

        double rDeadband = 5; //5cm

        // Calculate how far away we are from the goal
        double currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));
        double powerTurn = 1.0;
        if (opMode.isStopRequested()) {
            return;
        } else {
            while (opMode.opModeIsActive() && rDeadband < currentR || Math.abs(goal.getHeading() - getPosition().getHeading()) > 2) {
                if (opMode.isStopRequested()) {
                    break;
                }
                // Position logging in a CSV file
                String CSV = getPosition().getX() + "," + getPosition().getY() + "," + getPosition().getHeading();
                logFile.println(CSV);

                // Regenerate the current R
                currentR = Math.sqrt(Math.pow(goal.getX() - getPosition().getX(), 2) + Math.pow(goal.getY() - getPosition().getY(), 2));

                telemetry.addData("Current R", currentR);
                telemetry.addData("Angle Error", Math.abs(goal.getHeading() - getPosition().getHeading()));

                // Use PID to find the X Y and C powers
                double xVal = pidControllerX.getError(getPosition().getX(), goal.getX(), 0.5);
                double yVal = pidControllerY.getError(getPosition().getY(), goal.getY(), 2.0);
                double cVal = pidControllerC.getError(getPosition().getHeading(), goal.getHeading(), 2.0);

                // Prevent the turn from going lower than 0.2
                if (cVal < 0.2 && cVal > -0.2) {
                    if (cVal < 0) {
                        cVal = -0.2;
                    } else {
                        cVal = 0.2;
                    }
                }
                // Apply the powers
                fieldOriented(yVal, -xVal * (2), cVal * powerTurn, power, getPosition().getHeading());

                // Update position
                update();
            }
            moveBy(0, 0, 0, 0);
        }
    }

    /**
     * Turns to an angle
     *
     * @param goal
     * @param power
     * @param opMode1
     */
    public void turn(double goal, double power, LinearOpMode opMode1) {
        double error;

        // Check to see whether to turn left or right
        if (Math.abs(goal - getPosition().getHeading()) < Math.abs(360 - goal + getPosition().getHeading())) {
            if (goal - getPosition().getHeading() < 0) {
                power = Math.abs(power);
            } else {
                power = -Math.abs(power);
            }
        } else {
            if (360 - goal + getPosition().getHeading() > 0) {
                power = Math.abs(power);
            } else {
                power = -Math.abs(power);
            }
        }
        while (opMode.opModeIsActive() && (Math.abs(goal - getPosition().getHeading()) > 1)) {
            if (!opMode.opModeIsActive()) return;
            // Check to see whether to turn left or right
            // Check to see whether to turn left or right
            if (Math.abs(goal - getPosition().getHeading()) < Math.abs(360 - goal + getPosition().getHeading())) {
                if (goal - getPosition().getHeading() < 0) {
                    power = Math.abs(power);
                } else {
                    power = -Math.abs(power);
                }
            } else {
                if (360 - goal + getPosition().getHeading() > 0) {
                    power = Math.abs(power);
                } else {
                    power = -Math.abs(power);
                }
            }

            // Calculate error with PID
            error = pidControllerC.getError(getPosition().getHeading(), goal, 0.5);

            // Prevent turn speed from getting under 0.5
            if (error <= 0.5) {
                error = 0.5;
            }
            // Apply turn power
            moveBy(0, 0, error, power);

            // Update odometry
            update();
        }
        moveBy(0, 0, 0, 0);
    }

    /**
     * Generic Tele-Op move function
     *
     * @param y
     * @param x
     * @param c
     * @param DRIVE_REDUC
     */
    public void moveBy(double y, double x, double c, double DRIVE_REDUC) {
        //Joystick deadzones
        if (Math.abs(y) <= 0.05) {
            y = 0;
        }
        if (Math.abs(x) <= 0.05) {
            x = 0;
        }
        if (Math.abs(c) <= 0.05) {
            c = 0;
        }

        double FLval = -(y - x) + c;
        double FRval = -(y + x) - c;
        double BLval = -(y + x) + c;
        double BRval = -(y - x) - c;

        double FLabs = Math.abs(FLval);
        double FRabs = Math.abs(FRval);
        double BLabs = Math.abs(BLval);
        double BRabs = Math.abs(BRval);

        double maxPower = 1;

        if (FLabs > maxPower) {
            maxPower = FRabs;
        }
        if (FRabs > maxPower) {
            maxPower = FRabs;
        }
        if (BLabs > maxPower) {
            maxPower = BLabs;
        }
        if (BRabs > maxPower) {
            maxPower = BRabs;
        }

        FLval /= maxPower;
        FRval /= maxPower;
        BLval /= maxPower;
        BRval /= maxPower;

        //Set power to the wheels
        frontLeft.setPower(Range.clip((FLval * DRIVE_REDUC), -1, 1));
        frontRight.setPower(Range.clip(-(FRval * DRIVE_REDUC), -1, 1));
        backLeft.setPower(Range.clip((BLval * DRIVE_REDUC), -1, 1));
        backRight.setPower(Range.clip(-(BRval * DRIVE_REDUC), -1, 1));

        String CSV = getPosition().getX() + "," + getPosition().getY() + "," + getPosition().getHeading();
        logFile.println(CSV);

        telemetry.addData("yOut", y);
        telemetry.addData("XOut", x);
        //telemetry.addData("X", getPosition().getX());
        //telemetry.addData("Y", getPosition().getY());
        //telemetry.addData("C", getPosition().getHeading());

        //telemetry.addData("C_Raw", getRawCenterPosition());
        //telemetry.update();
    }

    /**
     * Field Centric drive funtion used in auto to arc smoothley
     *
     * @param y
     * @param x
     * @param c
     * @param driveReduc
     * @param angle
     */
    // 1 1
    public void fieldOriented(double y, double x, double c, double driveReduc, double angle) {
        double cosA = Math.cos(Math.toRadians(angle)); // 0.86602529158
        double sinA = Math.sin(Math.toRadians(angle)); // 0.5
        double xOut = x * cosA - y * sinA; //0.2
        double yOut = x * sinA + y * cosA; //1

        double adjustedStrafe;
        if (Math.abs(xOut) < 0.1 && Math.abs(xOut) > -0.1 && xOut != 0) {
            adjustedStrafe = 0.1 * (xOut / Math.abs(xOut));
        } else {
            adjustedStrafe = xOut;
        }


        moveBy(yOut, adjustedStrafe * 3, c, driveReduc);
    }

    /**
     * stops motors
     */
    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * resets drive encoders
     */
    public void reset() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setPower(0.0);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * creates a unique filename for logging
     *
     * @param fileName  orig filename
     * @param extension file extension
     * @return a unique filename
     */
    private String createUniqueFileName(String fileName, String extension) {
        return fileName + DateFormat.getDateTimeInstance().format(new Date()) + extension;
    }

    /**
     * Overides the method in TrigThreeWheelLocalizer so that the localizer can have access to the
     * encoder values of each tracking wheel.
     *
     * @return
     */
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                getLeftPosition(),
                getRightPosition(),
                getCenterPosition()
        );
    }
}

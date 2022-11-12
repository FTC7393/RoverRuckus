package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabber;

import java.io.File;
import java.io.PrintStream;
import java.text.DateFormat;
import java.util.Date;


@Autonomous(name = "Red 1", group = "red")
public class Red1 extends LinearOpMode {
    private FrameGrabber frameGrabber;
    private Robot robot;
    public ElapsedTime eTime;

    private boolean isRunning = true;

    private enum POSITION {
        LEFT,
        CENTER,
        RIGHT,
    }

    POSITION position;
    public double currentAngle;
    double lastXPos;
    private File file;
    private PrintStream logFile;
    private long delay;

    @Override
    public void runOpMode() throws InterruptedException {
        /*file = new File(Environment.getExternalStorageDirectory(), createUniqueFileName("test", ".txt") + "");

        try {
            logFile = new PrintStream(new FileOutputStream(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }*/


        robot = new Robot(this, hardwareMap, telemetry);
        eTime = new ElapsedTime();

        frameGrabber = new FrameGrabber();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        frameGrabber.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        frameGrabber.enable();

        while (!isStarted()) {
            // Left Guide
            if (gamepad1.dpad_right == true) {
                frameGrabber.leftGuide += 0.001;
            } else if (gamepad1.dpad_left == true) {
                frameGrabber.leftGuide -= 0.001;
            }

            // Mask
            if (gamepad1.dpad_down == true) {
                frameGrabber.mask += 0.001;
            } else if (gamepad1.dpad_up == true) {
                frameGrabber.mask -= 0.001;
            }

            // Threshold
            if (gamepad2.y) {
                frameGrabber.threshold += 0.001;
            } else if (gamepad2.a) {
                frameGrabber.threshold -= 0.001;
            }

            if (frameGrabber.position == "LEFT") {
                position = POSITION.LEFT;
            } else if (frameGrabber.position == "MIDDLE") {
                position = POSITION.CENTER;
            } else {
                position = POSITION.RIGHT;
            }
            telemetry.addData("Position", position);
            telemetry.addData("Threshold", frameGrabber.threshold);

            telemetry.update();
        }

        AutoTransitioner.transitionOnStop(this, "Drive");
        waitForStart();

        // START OF AUTO ===========================================================================

        frameGrabber.disable();
        robot.hang.resetEncoders();

        while (opModeIsActive() && robot.hang.getCurrentPosition() > -12000) {
            if (!opModeIsActive()) return;
            robot.hang.lift(-1);
        }
        robot.hang.lift(-0);

        delay(1);
        robot.mecanumDrive.reset();

        // Move forward of the lander
        robot.mecanumDrive.lineTo(new Pose2d(135, 135, -135.0), 0.7, this);

        // Turn towards the mineral
        delay(1);
        if (position == POSITION.LEFT) {
            currentAngle = -165.0;
            robot.mecanumDrive.turn(currentAngle, 1, this);
        } else if (position == POSITION.CENTER) {
            currentAngle = -135.0;
            robot.mecanumDrive.turn(currentAngle, 1, this);
        } else {
            currentAngle = -99.0;
            robot.mecanumDrive.turn(currentAngle, 1, this);
        }

        // Extend the arm while intaking to hit the mineral
        robot.craterArm.forward(40.0, this);
        delay(0.2);
        // Bring the arm back
        robot.craterArm.toBase();

        // Turn towards the wall
        robot.mecanumDrive.turn(-15.0, 1.0, this);
        robot.mecanumDrive.stopMotors();

        // Move towards the wall
        robot.mecanumDrive.lineTo(new Pose2d(60.0, 150.0, -15.0), 0.7, this);
        robot.mecanumDrive.stopMotors();

        // Arc towards the depot
        robot.mecanumDrive.arc(new Pose2d(10.0, 210.0, -85), 0.7, this);
        robot.mecanumDrive.stopMotors();

        // Move to the depot
        robot.mecanumDrive.lineTo(new Pose2d(5.0, 280.0, -85), 0.7, this);
        robot.mecanumDrive.stopMotors();

        // Deposit the marker
        robot.landerLift.deposit(true);
        delay(1.5);

        eTime.reset();
        while (opModeIsActive() && eTime.time() < 0.2) {
            robot.landerLift.lift(-1.0);
        }
        robot.landerLift.lift(0.0);

        // Move the the crater
        new Thread(new Runnable() {
            @Override
            public void run() {
                // Lower the han
                while (opModeIsActive() && robot.hang.getCurrentPosition() < -7300) {
                    robot.hang.lift(1);
                }
                robot.hang.lift(0);
            }
        }).start();
        // MIDPOINT CHANGE PLZ
        robot.mecanumDrive.lineTo(new Pose2d(7.0, 210, -85), 0.8, this);

        robot.mecanumDrive.lineTo(new Pose2d(20.0, 130.0, -85), 0.8, this);

        // Extend the arm to partially park to the mineral
        robot.craterArm.forwardNoIntake(45.0, this);

        robot.mecanumDrive.stopMotors();

        Thread intake = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    robot.craterArm.intakePower(true, 1);
                }
            }
        });
        intake.start();

        robot.craterArm.forward(55.0, this);
        delay(1.0);

        robot.craterArm.forward(70.0, this);
        delay(1.0);

        robot.craterArm.backward(55.0, this);
        delay(1.0);

        intake.interrupt();

    }

    public void delay(double seconds) {
        eTime.reset();
        while (eTime.time() < seconds && opModeIsActive()) ;
    }

    private String createUniqueFileName(String fileName, String extension) {
        return fileName + DateFormat.getDateTimeInstance().format(new Date()) + extension;
    }
}
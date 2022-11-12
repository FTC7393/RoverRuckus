package org.firstinspires.ftc.teamcode.OpMode.Tele;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;
import org.firstinspires.ftc.teamcode.Robot.Util.UTILToggle;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;
import java.text.DateFormat;
import java.util.Date;

@TeleOp(name = "FieldCentric")
public class FieldCentric extends LinearOpMode {
    private Robot robot;
    double startTime = 0, endTime = 0;
    double elapsedTime;
    private File file;
    private PrintStream logFile;
    boolean hasAPressed = false;
    boolean hasBPressed = false;
    Gamepad lastgamepad1 = new Gamepad();
    public ElapsedTime eTime;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        eTime = new ElapsedTime();
        UTILToggle flickOfDaWrist = new UTILToggle();    // Slows down drivetrain when on

        // Logging code
        file = new File(Environment.getExternalStorageDirectory(), createUniqueFileName("raw_wheel_pos", ".csv") + "");

        try {
            logFile = new PrintStream(new FileOutputStream(file));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        logFile.println("l,r,m");

        //robot.mecanumDrive.resetTeleop();

        robot.mecanumDrive.reset();
        waitForStart();
        Thread odometry = new Thread(new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //startTime = System.currentTimeMillis();
                    //elapsedTime = startTime - endTime;

                    robot.mecanumDrive.update();

                    //logFile.println(robot.mecanumDrive.getRawLeftPosition() + "," + robot.mecanumDrive.getRawRightPosition() + "," + robot.mecanumDrive.getRawCenterPosition());

                    //endTime = System.currentTimeMillis();
                }
            }
        });
        //odometry.start();
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("X", robot.mecanumDrive.getPosition().getX());
            telemetry.addData("Y", robot.mecanumDrive.getPosition().getY());
            telemetry.addData("C", robot.mecanumDrive.getPosition().getHeading());

            telemetry.addData("Left", robot.mecanumDrive.getRawLeftPosition());
            telemetry.addData("Center", robot.mecanumDrive.getRawCenterPosition());
            telemetry.addData("Right", robot.mecanumDrive.getRawRightPosition());

            telemetry.addData("FL", robot.mecanumDrive.frontLeft.getCurrentPosition());
            telemetry.addData("BL", robot.mecanumDrive.backLeft.getCurrentPosition());
            telemetry.addData("FR", robot.mecanumDrive.frontRight.getCurrentPosition());
            telemetry.addData("BR", robot.mecanumDrive.backRight.getCurrentPosition());

            telemetry.addData("Lift", robot.landerLift.lift.getCurrentPosition());

            telemetry.update();
            /////////////////////////////////////////////////////////////////////
            //=========================GAMEPAD 1 DRIVE=========================//
            /////////////////////////////////////////////////////////////////////
            //DRIVE
            if (gamepad1.a) {
                robot.mecanumDrive.frontLeft.setPower(0.6);
                robot.mecanumDrive.frontRight.setPower(0.6);
                robot.mecanumDrive.backRight.setPower(0.6);
                robot.mecanumDrive.backLeft.setPower(0);
                hasAPressed = true;

            } else if (gamepad1.b) {
                robot.mecanumDrive.frontLeft.setPower(-0.7);
                robot.mecanumDrive.frontRight.setPower(-0.7);
                robot.mecanumDrive.backRight.setPower(-0.7);
                robot.mecanumDrive.backLeft.setPower(0);
                hasBPressed = true;

            } else if (gamepad2.left_trigger > 0.5) {
                robot.mecanumDrive.fieldOriented(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad2.right_stick_x,
                        0.75,
                        robot.mecanumDrive.getPosition().getHeading()
                );
            } else {
                //Calls the drive function at a 50% slow powers
                robot.mecanumDrive.fieldOriented(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        1.0,
                        robot.mecanumDrive.getPosition().getHeading()
                );
            }

            try {
                lastgamepad1.copy(gamepad1);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            telemetry.addData("last a", lastgamepad1.a);


            if (flickOfDaWrist.status(gamepad1.a) == UTILToggle.Status.COMPLETE) {
                delay(0.1);
                robot.mecanumDrive.frontLeft.setPower(0.6);
                robot.mecanumDrive.frontRight.setPower(0.6);
                robot.mecanumDrive.backRight.setPower(0.6);
                robot.mecanumDrive.backLeft.setPower(0);
                delay(0.035);
            }

            //HANG
            if (gamepad1.left_trigger > 0.5) { //down
                robot.hang.lift(1);
            } else if (gamepad1.right_trigger > 0.5) { //up
                robot.hang.lift(-1);
            } else {
                robot.hang.lift(0);
            }


            /////////////////////////////////////////////////////////////////////
            //=========================GAMEPAD 2 DRIVE=========================//
            /////////////////////////////////////////////////////////////////////
            //LANDER LIFT

            if (gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right) {
                if (robot.landerLift.lift.getCurrentPosition() > -1078) {
                    if (robot.landerLift.lift.getCurrentPosition() > -1040) {
                        robot.landerLift.lift(1);
                    } else {
                        robot.landerLift.lift(0.5);
                    }
                } else {
                    if (robot.landerLift.lift.getCurrentPosition() > -10) {
                        robot.landerLift.lift(0);
                    } else {
                        robot.landerLift.lift(0.3);
                    }
                }
            } else if (gamepad2.a) {
                if (Math.abs(gamepad2.left_stick_y) > 0) {
                    robot.landerLift.lift(-gamepad2.left_stick_y);
                } else {
                    robot.landerLift.pureLift(0.3);
                }
            } else if (Math.abs(gamepad2.left_stick_y) > 0) {
                robot.landerLift.lift(-gamepad2.left_stick_y);
            } else {
                if (robot.landerLift.lift.getCurrentPosition() > -10) {
                    robot.landerLift.lift(-1.0);
                } else if (robot.landerLift.lift.getCurrentPosition() > -80) {
                    robot.landerLift.lift(-1.0);
                } else {
                    robot.landerLift.lift(-0.7);
                }
            }


            //DEPOSITOR
            if (gamepad1.left_bumper) {
                robot.landerLift.deposit(true);
            } else {
                robot.landerLift.deposit(false);
            }

            //CRATER ARM
            robot.craterArm.lift(gamepad2.right_stick_y);

            // OUTAKE/INTAKE
            if (gamepad2.right_trigger > 0.1) {
                robot.craterArm.intakePower(true, gamepad2.right_trigger);
            } else if ((gamepad2.right_bumper)) {
                robot.craterArm.outTake(true);
            } else {
                robot.craterArm.intake(false);
                robot.craterArm.outTake(false);
            }

            /*if (gamepad2.right_trigger > 0.1) {
                robot.craterArm.outTake(true);
            } else {
                robot.craterArm.outTake(false);
            }*/

            telemetry.update();
            robot.mecanumDrive.update();

        }

    }

    private String createUniqueFileName(String fileName, String extension) {
        return fileName + DateFormat.getDateTimeInstance().format(new Date()) + extension;
    }

    public void delay(double seconds) {
        eTime.reset();
        while (eTime.time() < seconds && opModeIsActive()) ;
    }
}

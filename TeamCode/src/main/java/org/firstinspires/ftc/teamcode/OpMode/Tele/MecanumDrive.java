package org.firstinspires.ftc.teamcode.OpMode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;

@TeleOp(name = "Drive")
public class MecanumDrive extends LinearOpMode {
    private Robot robot;
    boolean hasAPressed = false;
    boolean hasBPressed = false;
    public ElapsedTime eTime;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        eTime = new ElapsedTime();

        // Logging code

        robot.mecanumDrive.reset();
        waitForStart();

        robot.landerLift.reset();
        //robot.craterArm.reset();
        while (!isStopRequested()) {
            telemetry.addData("lift", robot.landerLift.lift.getCurrentPosition());
            telemetry.addData("pose", robot.mecanumDrive.getPosition());
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
                robot.mecanumDrive.moveBy(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad2.right_stick_x,
                        0.75
                );
            } else {
                //Calls the drive function at a 50% slow powers
                robot.mecanumDrive.moveBy(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x,
                        1.0
                );
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
                if (robot.landerLift.lift.getCurrentPosition() > -1157) {
                    if (robot.landerLift.lift.getCurrentPosition() > -1050) {
                        robot.landerLift.lift(1);
                    } else {
                        robot.landerLift.lift(0.7);
                    }
                } else {
                    robot.landerLift.lift(0.3);
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
                if (robot.landerLift.lift.getCurrentPosition() < 10) {
                    robot.landerLift.lift(0.0);
                }
                robot.landerLift.lift(-1.0);
            }

            if (robot.landerLift.limitSwitch.getState() == false) {
                robot.landerLift.reset();
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

            telemetry.update();
            //robot.mecanumDrive.update();
        }
    }
}

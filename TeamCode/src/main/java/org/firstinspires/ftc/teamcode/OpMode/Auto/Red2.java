package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.AutoTransitioner;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.Pose2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabber;

@Autonomous(name = "Red 2", group = "red")
public class Red2 extends LinearOpMode {
    private FrameGrabber frameGrabber;
    private Robot robot;
    private ElapsedTime eTime;

    private boolean isRunning = true;

    private enum POSITION {
        LEFT,
        CENTER,
        RIGHT,
    }

    POSITION position;
    boolean giantDiencephalicBrainStemRoboticsTeam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        eTime = new ElapsedTime();

        frameGrabber = new FrameGrabber();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        frameGrabber.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        frameGrabber.enable();

        //robot.landerLift.reset();
        //robot.craterArm.reset();
        while (!isStarted()) {
            // Left Guide
            if (gamepad1.dpad_right) {
                frameGrabber.leftGuide += 0.001;
            } else if (gamepad1.dpad_left) {
                frameGrabber.leftGuide -= 0.001;
            }

            // Mask
            if (gamepad1.dpad_down) {
                frameGrabber.mask += 0.001;
            } else if (gamepad1.dpad_up) {
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

            if (gamepad1.b) {
                giantDiencephalicBrainStemRoboticsTeam = giantDiencephalicBrainStemRoboticsTeam ? false : true;
            }
            telemetry.addData("Position", position);
            telemetry.addData("Threshold", frameGrabber.threshold);
            telemetry.addData("Giant Diencephalic Brain Stem Robotics Team Auto: ", giantDiencephalicBrainStemRoboticsTeam);

            telemetry.update();
        }

        AutoTransitioner.transitionOnStop(this, "Drive");
        waitForStart();

        if (opModeIsActive()) {
            robot.landerLift.deposit(false);
            try {
                {
                    //    _____ __             __           ____   ___         __
                    //   / ___// /_____ ______/ /_   ____  / __/  /   | __  __/ /_____
                    //   \__ \/ __/ __ `/ ___/ __/  / __ \/ /_   / /| |/ / / / __/ __ \
                    //  ___/ / /_/ /_/ / /  / /_   / /_/ / __/  / ___ / /_/ / /_/ /_/ /
                    // /____/\__/\__,_/_/   \__/   \____/_/    /_/  |_\__,_/\__/\____/

                    frameGrabber.disable();
                    robot.hang.resetEncoders();
                    //robot.craterArm.reset();

                    while (opModeIsActive() && robot.hang.getCurrentPosition() > -12000) {
                        if (!opModeIsActive()) return;
                        robot.hang.lift(-1);
                    }
                    robot.hang.lift(-0);

                    delay(0.25);
                    robot.mecanumDrive.reset();

                    ////////////////////////////////////////////////////////////////////////////////////////////////
                    robot.mecanumDrive.lineTo(new Pose2d(-130, 130, -45), 0.5, this);
                    ////////////////////////////////////////////////////////////////////////////////////////////////

                    if (giantDiencephalicBrainStemRoboticsTeam) {
                        delay(3);
                    } else if (position == POSITION.RIGHT) {
                        delay(1);
                    }
                    if (position != POSITION.CENTER) {
                        // Turn towards mineral
                        if (position == POSITION.LEFT) {
                            robot.mecanumDrive.turn(-13, 1.0, this);
                        } else {
                            robot.mecanumDrive.turn(-80, 1.0, this);
                        }

                        // knock of mineral
                        robot.craterArm.forwardNoIntake(50.0, this);
                        delay(0.2);

                        // turn to center
                        if (position == POSITION.LEFT) {
                            robot.mecanumDrive.turn(-45, 1.0, this);
                        } else if (position == POSITION.CENTER) {
                        } else {
                            robot.mecanumDrive.turn(-45, 1.0, this);
                        }
                        // bring arm to depot and shoot our marker

                        robot.craterArm.forwardNoIntake(80.0, this);
                        robot.craterArm.intake(true);
                        delay(0.25);
                        robot.craterArm.intake(false);

                        robot.craterArm.backwardNoIntake(80.0, this);

                        // turn back to the mineral
                        if (position == POSITION.LEFT) {
                            robot.mecanumDrive.turn(-10, 1.0, this);
                        } else if (position == POSITION.CENTER) {
                        } else {
                            robot.mecanumDrive.turn(-80, 1.0, this);
                        }
                        //bring arm back
                        delay(0.5);
                        robot.craterArm.toBase();
                    } else {
                        robot.craterArm.forwardNoIntake(80.0, 1.0, this);
                        robot.craterArm.intake(true);
                        delay(0.5);
                        robot.craterArm.intake(false);

                        robot.craterArm.toBase();
                    }

                    //===============================================================================

                    // turn towards wall
                    robot.mecanumDrive.turn(25, 1.0, this);
                    Thread lowerHang = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            // Lower the han
                            while (opModeIsActive() && robot.hang.getCurrentPosition() < -7300) {
                                robot.hang.lift(1);
                            }
                            robot.hang.lift(0);
                        }
                    });
                    lowerHang.start();

                    // move towards wall and turn towards crater
                    robot.mecanumDrive.lineTo(
                            new Pose2d(-60, 180, 25),
                            0.6,
                            20,
                            this
                    );
                    robot.mecanumDrive.turn(75, 1.0, this);

                    //stick arm into crater and intake
                    if (position != POSITION.CENTER) {
                        pickUpBlocksLong();
                    } else {
                        pickUpBlocks();
                    }

                    // align with lander
                    robot.mecanumDrive.lineToTime(
                            new Pose2d(-80, 183, 75),
                            1.0,
                            10,
                            1.5,
                            this
                    );

                    //lift arm
                    Thread hold = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            // Lower the han
                            while (opModeIsActive()) {
                                robot.landerLift.lift(0.3);
                            }
                            robot.landerLift.lift(0);
                        }
                    });
                    liftNhold(hold);

                    // point turn to score
                    while (opModeIsActive() && robot.mecanumDrive.getPosition().getHeading() > 15) {
                        robot.mecanumDrive.frontLeft.setPower(0.6);
                        robot.mecanumDrive.frontRight.setPower(0.6);
                        robot.mecanumDrive.backRight.setPower(0.6);
                        robot.mecanumDrive.backLeft.setPower(0);
                        robot.mecanumDrive.update();
                    }
                    robot.mecanumDrive.stopMotors();
                    delay(0.1);
                    robot.mecanumDrive.frontLeft.setPower(0.6);
                    robot.mecanumDrive.frontRight.setPower(0.6);
                    robot.mecanumDrive.backRight.setPower(0.6);
                    robot.mecanumDrive.backLeft.setPower(0);
                    delay(0.035);
                    robot.mecanumDrive.stopMotors();

                    // deposit
                    robot.landerLift.deposit(true);
                    delay(1.0);

                    // move away from lander
                    robot.mecanumDrive.lineTo(
                            new Pose2d(-55, 183, 15),
                            1.0,
                            20,
                            this
                    );
                    robot.landerLift.deposit(false);

                    // stop holding
                    hold.interrupt();

                    // bring lift down
                    Thread down = new Thread(new Runnable() {
                        @Override
                        public void run() {
                            // Lower the han
                            while (opModeIsActive()) {
                                robot.landerLift.lift(-0.7);
                            }
                            robot.landerLift.lift(0);
                        }
                    });
                    down.start();

                    // turn towards crater
                    robot.mecanumDrive.turn(60, 1.0, this);

                    // stick arm into crater
                    robot.craterArm.forward(90.0, this);
                    down.interrupt();

                    // ========================================================================================
                    // Optional multi cycle ===================================================================
                    // ========================================================================================
                    if (position == POSITION.CENTER) {
                        pickUpBlocks2();
                        // align with lander
                        robot.mecanumDrive.lineToTime(
                                new Pose2d(-80, 183, 75),
                                1.0,
                                10,
                                1.5,
                                this
                        );

                        //lift arm
                        Thread hold1 = new Thread(new Runnable() {
                            @Override
                            public void run() {
                                // Lower the han
                                while (opModeIsActive()) {
                                    robot.landerLift.lift(0.3);
                                }
                                robot.landerLift.lift(0);
                            }
                        });
                        liftNhold(hold1);

                        // point turn to score
                        while (opModeIsActive() && robot.mecanumDrive.getPosition().getHeading() > 10) {
                            robot.mecanumDrive.frontLeft.setPower(0.6);
                            robot.mecanumDrive.frontRight.setPower(0.6);
                            robot.mecanumDrive.backRight.setPower(0.6);
                            robot.mecanumDrive.backLeft.setPower(0);
                            robot.mecanumDrive.update();
                        }
                        robot.mecanumDrive.stopMotors();
                        delay(0.1);
                        robot.mecanumDrive.frontLeft.setPower(0.6);
                        robot.mecanumDrive.frontRight.setPower(0.6);
                        robot.mecanumDrive.backRight.setPower(0.6);
                        robot.mecanumDrive.backLeft.setPower(0);
                        delay(0.035);
                        robot.mecanumDrive.stopMotors();

                        robot.landerLift.deposit(true);
                        delay(1);

                        // move away from lander
                        robot.mecanumDrive.lineTo(
                                new Pose2d(-55, 183, 10),
                                1.0,
                                20,
                                this
                        );

                        // stop holding
                        hold1.interrupt();

                        // bring lift down
                        Thread down1 = new Thread(new Runnable() {
                            @Override
                            public void run() {
                                // Lower the han
                                while (opModeIsActive()) {
                                    robot.landerLift.lift(-0.7);
                                }
                                robot.landerLift.lift(0);
                            }
                        });
                        down1.start();

                        // turn towards crater
                        robot.mecanumDrive.turn(50, 1.0, this);

                        // stick arm into crater
                        robot.craterArm.forward(90.0, this);
                        down1.interrupt();
                    }
                }
            } catch (Exception e) {
                telemetry.addData("ffffff", "ffffff");
            }
        }
    }


    public void pickUpBlocks() {
        robot.craterArm.forward(30.0, 1.0, this);
        robot.craterArm.forward(80.0, 0.5, this);
        delay(1.0);
        robot.craterArm.outTake(true);
        delay(0.25);
        robot.craterArm.outTake(false);
        robot.craterArm.toBase();

        // robot.craterArm.reset();
    }

    public void pickUpBlocksLong() {
        robot.craterArm.forward(30.0, 1.0, this);
        robot.craterArm.forward(80.0, 0.5, this);
        delay(3.5);
        robot.craterArm.outTake(true);
        delay(0.75);
        robot.craterArm.outTake(false);
        robot.craterArm.toBase();

        // robot.craterArm.reset();
    }

    public void pickUpBlocks2() {
        //robot.craterArm.forward(5.0, 1.0, this);
        robot.craterArm.forward(4.0, 0.5, this);
        delay(1.0);
        robot.craterArm.outTake(true);
        delay(0.25);
        robot.craterArm.outTake(false);
        robot.craterArm.toBase();
    }


    public void liftNhold(Thread hold) {
        while (robot.landerLift.lift.getCurrentPosition() > -1157) {
            if (robot.landerLift.lift.getCurrentPosition() > -1050) {
                robot.landerLift.lift(1);
            } else {
                robot.landerLift.lift(0.7);
            }
        }

        hold.start();
    }

    private void delay(double seconds) {
        eTime.reset();
        while (eTime.time() < seconds && opModeIsActive()) {
            telemetry.addData("f", "");
        }

    }
}
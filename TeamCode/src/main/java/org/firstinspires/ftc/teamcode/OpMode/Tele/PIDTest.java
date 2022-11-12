package org.firstinspires.ftc.teamcode.OpMode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;

@Disabled
@Autonomous(name = "PID Test")
public class PIDTest extends LinearOpMode {
    private Robot robot;
    ElapsedTime eTime;
    boolean isRunning = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        robot.mecanumDrive.reset();
        eTime = new ElapsedTime();

        waitForStart();

        while (isRunning && opModeIsActive()) {
            while (opModeIsActive()) {
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        while (opModeIsActive()) {
                            robot.craterArm.intakePower(true, 1);
                        }
                    }
                }).start();

                robot.craterArm.forward(55.0, this);
                delay(1.0);
                robot.craterArm.backward(45.0, this);
                delay(1.0);
            }
        }
        robot.mecanumDrive.stopMotors();
    }

    public void delay(double seconds) {
        eTime.reset();
        while (eTime.time() < seconds && opModeIsActive()) ;
    }

}
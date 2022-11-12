package org.firstinspires.ftc.teamcode.OpMode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;
import org.firstinspires.ftc.teamcode.vision.FrameGrabber;


@Autonomous(name = "test", group = "red")
public class test extends LinearOpMode {
    private FrameGrabber frameGrabber;
    private Robot robot;
    public ElapsedTime eTime;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);
        eTime = new ElapsedTime();

        waitForStart();

        // START OF AUTO ===========================================================================

        robot.hang.resetEncoders();

        while (opModeIsActive() && robot.hang.getCurrentPosition() > -12000) {
            robot.hang.lift(-1);
        }
    }
}
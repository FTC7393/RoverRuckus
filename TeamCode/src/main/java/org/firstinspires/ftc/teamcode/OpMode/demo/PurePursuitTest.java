package org.firstinspires.ftc.teamcode.OpMode.demo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.PurePursuit;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry.Vector2d;
import org.firstinspires.ftc.teamcode.Robot.Subsystem.Robot;

@Disabled
@Autonomous(name = "Pure Pursuit", group = "demo")
public class PurePursuitTest extends LinearOpMode {
    double[][] path = {
            {0, 0},
            {-143, 143},
            {-122, 122},
            {-21, 174},
            {-21, 210}
    };

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, hardwareMap, telemetry);

        path = PurePursuit.inject(path, 100);

        waitForStart();

        // START OF AUTO ===========================================================================
        while (opModeIsActive()) {
            telemetry.addData("pose: ", robot.mecanumDrive.getPosition());
            Vector2d powers = PurePursuit.loop(path, robot.mecanumDrive.getPosition().getX(), robot.mecanumDrive.getPosition().getY(), robot.mecanumDrive.getPosition().getHeading(), 10);
            telemetry.addData("powers", powers);
            telemetry.addData("lookahead", PurePursuit.getLookAheadPoint(path, robot.mecanumDrive.getPosition(), 10));

            robot.mecanumDrive.fieldOriented(powers.getX(), powers.getY(), 0, 0.5, robot.mecanumDrive.getPosition().getHeading());

            //robot.mecanumDrive.setSkidSteerPowers(PurePursuit.loop(path, robot.mecanumDrive.getPosition().getX(), robot.mecanumDrive.getPosition().getY(), robot.mecanumDrive.getPosition().getHeading(), 15));

            robot.mecanumDrive.update();
            telemetry.update();
        }
        //robot.mecanumDrive.lineTo(new Pose2d(-122, 122, 135), 0.8, this);
        //robot.mecanumDrive.turn(55, 0.5, this);
    }
}
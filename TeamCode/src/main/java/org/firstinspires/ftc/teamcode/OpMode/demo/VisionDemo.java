package org.firstinspires.ftc.teamcode.OpMode.demo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.vision.FrameGrabber;

@TeleOp(name = "Example: Blue Vision Demo")
public class VisionDemo extends OpMode {
    private FrameGrabber frameGrabber;

    @Override
    public void init() {
        frameGrabber = new FrameGrabber();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        frameGrabber.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system
        frameGrabber.enable();
    }

    @Override
    public void init_loop() {
        // Left Guide
        if (gamepad1.dpad_right == true) {
            frameGrabber.leftGuide += 1;
        } else if (gamepad1.dpad_left == true) {
            frameGrabber.leftGuide -= 1;
        }

        // Mask
        if (gamepad1.dpad_down == true) {
            frameGrabber.mask++;
        } else if (gamepad1.dpad_up == true) {
            frameGrabber.mask--;
        }

        // Threshold
        if (gamepad2.y) {
            frameGrabber.threshold += 1;
        } else if(gamepad2.a) {
            frameGrabber.threshold -= 1;
        }
        telemetry.addData("Position", frameGrabber.position);
        telemetry.addData("Threshold", frameGrabber.threshold);
        telemetry.update();
    }

    @Override
    public void loop() {
    }

    public void stop() {
        // stop the vision system
        frameGrabber.disable();
    }
}
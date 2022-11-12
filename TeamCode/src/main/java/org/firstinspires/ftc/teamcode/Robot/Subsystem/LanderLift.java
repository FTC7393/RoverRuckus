package org.firstinspires.ftc.teamcode.Robot.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LanderLift extends Subsystem {
    public static DcMotor lift;
    private Servo depositer;
    public DigitalChannel limitSwitch;
    Telemetry telemetry;

    public LanderLift(Robot robot, HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        lift = map.get(DcMotor.class, "VS");
        depositer = map.get(Servo.class, "SD");
        limitSwitch = map.get(DigitalChannel.class, "VT");

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Lifts the lift at a certain power and will stop it from going down if the limit switch is hit
     *
     * @param power double between -1 and 1
     */
    public void lift(double power) {
        //telemetry.addData("Lander Lift", limitSwitch.getState());
        if (power < 0 && limitSwitch.getState() == false) {
            lift.setPower(0);
        } else {
            lift.setPower(Range.clip(-power, -1.0, 0.5));
        }
    }

    /**
     * lifts the lift at a certain power and ignores the touch sensor
     *
     * @param power double between -1 and 1
     */
    public void pureLift(double power) {
        lift.setPower(Range.clip(-power, -1.0, 0.5));
    }

    /**
     * Opens the depositor
     *
     * @param open true to open, false to close
     */
    public void deposit(boolean open) {
        if (open == true) {
            depositer.setPosition(0.5);
        } else {
            depositer.setPosition(1.0);
        }
    }

    /**
     * resets the encoder of the lift.
     */
    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

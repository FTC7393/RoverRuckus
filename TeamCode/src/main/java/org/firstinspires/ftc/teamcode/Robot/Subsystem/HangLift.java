package org.firstinspires.ftc.teamcode.Robot.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class HangLift extends Subsystem {
    public static DcMotor liftMotor;
    private TouchSensor limitSwitch;

    private DigitalChannel magLimitSwitch;

    public HangLift(Robot robot, HardwareMap map) {
        liftMotor = map.get(DcMotor.class, "H");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magLimitSwitch = map.get(DigitalChannel.class, "ML");
        magLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public double getCurrentPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void resetEncoders() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void lift(double power) {
        liftMotor.setPower(power);
    }

    public void lowerToTele() {
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(-7300);

        while (!liftMotor.isBusy()) {
            lift(1);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isPressed() {
        return magLimitSwitch.getState();
    }

    private boolean hasReached(double goal) {
        if (getCurrentPosition() < goal) {
            return false;
        } else {
            return true;
        }
    }
}

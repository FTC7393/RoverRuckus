package org.firstinspires.ftc.teamcode.Robot.Subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Util.PIDController;

public class CraterArm extends Subsystem {
    public DcMotor lift;
    public static DcMotor intake;
    private DigitalChannel limitSwitch;

    Telemetry telemetry;
    PIDController liftController;
    LinearOpMode opMode;

    CraterArm(Robot robot, HardwareMap map, Telemetry telemetry, LinearOpMode opMode) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        lift = map.get(DcMotor.class, "CL");
        intake = map.get(DcMotor.class, "I");
        limitSwitch = map.get(DigitalChannel.class, "sensor_digital");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        double errorAt100 = 150;

        liftController = new PIDController(1 / errorAt100, 0, 0);
    }

    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * @return centimeters
     */
    public double getPosition() {
        double cmPerEnc = 0.10995934659;

        return lift.getCurrentPosition() * cmPerEnc;
    }

    public void lift(double power) {
        //telemetry.addData("CraterArm", limitSwitch.getState());
        //telemetry.addData("power", lift.getPower());
        if (power > 0 && limitSwitch.getState() == false) {
            lift.setPower(0);
        } else {
            lift.setPower(-power);
        }
    }

    public void forward(double goal, LinearOpMode opMode1) {

        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (getPosition() < goal && opMode.opModeIsActive()) {
            if (!opMode.opModeIsActive()) return;
            lift.setPower(0.45);
            intakePower(true, 1);
        }
        lift.setPower(0);
    }


    public void forward(double goal, double power, LinearOpMode opMode1) {
        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (getPosition() < goal && opMode.opModeIsActive()) {
            if (!opMode.opModeIsActive()) return;
            lift.setPower(power);
            intakePower(true, 1);
        }
        lift.setPower(0);
    }

    public void backward(double goal, LinearOpMode opMode1) {
        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (getPosition() > goal && opMode.opModeIsActive()) {
            if (!opMode.opModeIsActive()) return;
            lift.setPower(-0.45);
            intakePower(true, 1);
        }
        lift.setPower(0);
    }

    public void backwardNoIntake(double goal, LinearOpMode opMode1) {
        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (getPosition() > goal && opMode.opModeIsActive()) {
            if (!opMode.opModeIsActive()) return;
            lift.setPower(-0.45);
            intakePower(false, 1);
        }
        lift.setPower(0);
    }


    public void forwardNoIntake(double goal, LinearOpMode opMode1) {
        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (!opMode.isStopRequested() && opMode.opModeIsActive() && getPosition() < goal) {
            if (!opMode.opModeIsActive()) return;
            lift.setPower(0.45);
        }
        lift.setPower(0);
    }

    public void forwardNoIntake(double goal, double power, LinearOpMode opMode1) {
        telemetry.addData("CraterArm", limitSwitch.getState());
        telemetry.addData("power", lift.getPower());

        while (opMode.opModeIsActive() && getPosition() < goal) {
            if (!opMode.opModeIsActive()) return;

            lift.setPower(power);
        }
        lift.setPower(0);
    }

    public void toBase() {
        while (limitSwitch.getState() != false) {
            lift.setPower(-1.0);
            //intakePower(true, 1);
        }
        lift.setPower(0);
        //intakePower(false, 0);
    }

    public void toBaseIntake() {
        while (limitSwitch.getState() != false) {

            lift.setPower(-0.45);
            intakePower(true, 1);
        }
        lift.setPower(0);
        intakePower(false, 0);
    }

    public void toBaseOutake() {
        while (limitSwitch.getState() != false) {
            lift.setPower(-0.45);
            outTake(true);
        }
        lift.setPower(0);
        outTake(false);
    }

    public void intake(boolean on) {
        if (on) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }

    public void intakePower(boolean on, double power) {
        if (on) {
            intake.setPower(-power);
        } else {
            intake.setPower(0);
        }
    }

    public void outTake(boolean on) {
        if (on) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
}

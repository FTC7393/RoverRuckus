package org.firstinspires.ftc.teamcode.Robot.Subsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.teamcode.Robot.Hardware.CachingMotor;

import java.util.ArrayList;
import java.util.List;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    private final List<Subsystem> subsystems;
    private final List<Subsystem> subsystemsWithProblem;

    public HangLift hang;
    public MecanumDrive mecanumDrive;
    public LanderLift landerLift;
    public CraterArm craterArm;

    private List<CachingMotor> motors;

    private OpModeManagerImpl opModeManager;
    private Thread subsystemExecutor, motorExecutor, telemetryExecutor;

    private OpMode master;

    public Robot(LinearOpMode opMode, HardwareMap map, Telemetry telemetry) {
        this.master = opMode;
        motors = new ArrayList<>();
        subsystems = new ArrayList<>();
        subsystemsWithProblem = new ArrayList<>();

        hang = new HangLift(this, map);
        mecanumDrive = new MecanumDrive(this, map, telemetry, opMode);
        landerLift = new LanderLift(this, map, telemetry);
        craterArm = new CraterArm(this, map, telemetry, opMode);

        subsystemExecutor = new Thread(new Runnable() {
            @Override
            public void run() {
                while (!Thread.currentThread().isInterrupted()) {
                    updateSubsystems();
                }
            }
        });
        subsystemExecutor.start();


    }

    private void updateMotors() {
        for (CachingMotor motor : motors) {
            motor.update();
        }
    }

    public void updateSubsystems() {
        updateMotors();
    }

    public void stop() {
        subsystemExecutor.interrupt();
    }


    protected void addMotor(CachingMotor motor) {
        motors.add(motor);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        stop();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }

    @Override
    public String getGlobalWarning() {
        StringBuilder builder = new StringBuilder();
        for (Subsystem subsystem : subsystemsWithProblem) {
            builder.append(subsystem.getClass().getName());
            builder.append('\n');
        }
        return builder.toString();
    }

    @Override
    public void setGlobalWarning(String warning) {

    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblem) {
            subsystemsWithProblem.clear();
        }
    }

    /*public void waitForAllSubsystems() {
        for (; ; ) {
            boolean complete = true;
            for (Subsystem subsystem : subsystems) {
                if (subsystem.isBusy()) complete = false;
            }
            if (complete) return;
            try {
                Thread.sleep(50);
            } catch (InterruptedException ie) {
                //yikes
            }
        }
    }*/
}


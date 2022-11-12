package org.firstinspires.ftc.teamcode.Robot.Subsystem;

public abstract class Subsystem {
    public boolean isBusy() {
        return false;
    }

    public void waitForCompleteion() {
        for (; ; ) {
            if (!isBusy()) return;
            try {
                Thread.sleep(50);
            } catch (InterruptedException ie) {
                //yikes
            }
        }
    }
}



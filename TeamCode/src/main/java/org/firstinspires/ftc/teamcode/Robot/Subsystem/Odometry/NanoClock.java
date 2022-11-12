package org.firstinspires.ftc.teamcode.Robot.Subsystem.Odometry;

public class NanoClock {
    public double seconds() {
        return System.nanoTime() / 1e9;
    }
}

package org.firstinspires.ftc.teamcode.Robot.Util;

import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

/**
 * This is a generic PID controller
 */
public class PIDController {
    private double kP, kI, kD;
    private double last, errorIntegral, errorDeriv, error;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        last = 0;
        errorIntegral = 0;
        errorDeriv = 0;
    }

    public double getError(double current, double goal, double limit) {
        errorDeriv = current - last;
        errorIntegral += error;

        last = current;

        // this is so hacky, im just pressed for time plz forgive
        /*if (goal == 0) {
            error = goal - current;
        } else {
            error = (goal - current) / goal;
        }*/

        // -80 - - 81
        error = goal - current;

        if (Math.abs(error) < limit) {
            return 0;
        } else {
            if (((kP * error) + (kI * errorIntegral) - (kD * errorDeriv)) < 0) {
                return Range.clip(((kP * error) + (kI * errorIntegral) - (kD * errorDeriv)), -1.0, -0.25);
            } else {
                return Range.clip(((kP * error) + (kI * errorIntegral) - (kD * errorDeriv)), 0.25, 1.0);
            }
        }
    }
}

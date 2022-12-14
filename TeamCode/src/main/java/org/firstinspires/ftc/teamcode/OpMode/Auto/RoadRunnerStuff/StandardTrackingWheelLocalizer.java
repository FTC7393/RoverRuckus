package org.firstinspires.ftc.teamcode.OpMode.Auto.RoadRunnerStuff;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // in; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Vector2d(0, LATERAL_DISTANCE / 2), // left
                new Vector2d(0, -LATERAL_DISTANCE / 2), // right
                new Vector2d(FORWARD_OFFSET, 0) // front
        ), Arrays.asList(0.0, 0.0, Math.PI / 2));

        leftEncoder = hardwareMap.get(DcMotor.class, "H");
        rightEncoder = hardwareMap.get(DcMotor.class, "BR");
        frontEncoder = hardwareMap.get(DcMotor.class, "I");
    }

    public static double encoderTicksToInches(int ticks) {
        return ((Math.PI * 60.54) / 1440 * ticks) / 10;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                -encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}

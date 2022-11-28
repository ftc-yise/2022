package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

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
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    // mpdavy 2022.11.27 --
    // TICKS_PER_REV comes from the Rev through-bore encoder docs
    public static double TICKS_PER_REV = 8192;
    // mpdavy 2022.11.27 --
    // Physically measuring the wheels does NOT work. Get the size from the vendor docs
    public static double WHEEL_RADIUS = 0.75; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.25; // in; distance between the left and right wheels
    // mpdavy 2022.11.27 --
    // Use a tape measure to find the center point between front and back wheels. Mark this point
    // then measure the distance from center point to the middle of the rear odometry wheel. If the
    // wheel is behind the center point, use the negative value
    public static double FORWARD_OFFSET = -8; // in; offset of the lateral wheel

    // mpdavy 2022.11.27 --
    // These values are calculated by measuring a fixed distance (ie 90 inches), manually pushing
    // the robot the measured distance at least 3 times, finding the average calculated distance
    // recorded in the telemetry and dividing the measured distance (ie 90) by the average
    // calculated distance
    public static double X_MULTIPLIER = 0.997; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1.145; // Multiplier in the Y direction

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_front_drive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_back_drive"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_front_drive"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)\
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    // mpdavy 2022.11.27 --
    // getRawVelocity was replaced with getCorrectedVelocity to deal with the Rev encoders use of
    // a 16-bit counter instead of a 32-bit counter which causes the counter to wrap very quickly
    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}

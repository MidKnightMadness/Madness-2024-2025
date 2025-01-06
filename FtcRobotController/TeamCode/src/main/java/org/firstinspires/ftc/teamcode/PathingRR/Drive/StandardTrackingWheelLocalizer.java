package org.firstinspires.ftc.teamcode.PathingRR.Drive;



import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.Drive.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.PathingRR.Util.Encoder;

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
public class StandardTrackingWheelLocalizer implements Localizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 32 / 25.4; // mm-> in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = DriveConstants.LATERAL_DISTANCE; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = DriveConstants.FORWARD_OFFSET; // in; offset of the lateral wheel

    private Pose2d poseEstimate;
    private Pose2d poseVelocity;
    public ThreeWheelOdometry odometry;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, double[] startingPosition) {

        odometry = new ThreeWheelOdometry(hardwareMap, startingPosition);

    }

    public void update(){
        odometry.update();
        poseEstimate = getPoseEstimate();
        poseVelocity = getPoseVelocity();
    }

    public Pose2d getPoseEstimate(){
        return new Pose2d(odometry.getXCoordinate(), odometry.getYCoordinate(), odometry.getRotationRadians());
    }


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {

    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        List<Double> wheelVelocities = odometry.getWheelVelocities();  // Get wheel velocities (may need to implement this in odometry)

        // Convert wheel velocities to robot velocities using kinematics
        Pose2d robotVelocities = MecanumKinematics.wheelToRobotVelocities(
                wheelVelocities,
                odometry.getTrackWidth(),
                odometry.getWheelBase(),
                odometry.getLateralMultiplier()
        );

        return new Pose2d(robotVelocities.getX(), robotVelocities.getY(), robotVelocities.getHeading());

    }
}

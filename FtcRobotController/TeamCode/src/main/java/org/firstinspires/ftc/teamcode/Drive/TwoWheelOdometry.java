package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.Helper.Timer;
import org.firstinspires.ftc.teamcode.PathingRR.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleOpModes.TrackingWheelLateralDistanceTuner;


import java.util.Arrays;
import java.util.List;

public class TwoWheelOdometry {
    IMUWrapper imu;
    public DcMotor yEncoder;
    public DcMotor xEncoder;
    Timer timer;

    double LateralMultiplier = 1;

    final double WHEEL_RADIUS_MM = 16;
    final double CENTER_ODOMETRY_DISTANCE_MM = 32;
    final double VERTICAL_ODOMETRY_DISTANCE_MM = 138.7896;

    final double CENTER_ARC_90_DEG = 2.7943;
    final double VERTICAL_ARC_90_DEG = 10.7398;

    final double IN_PER_MM = 0.0393701;

    final double trackDistance = 12.1;//in //11 for old chassis
    final double distVertEncoders = 4.2;//in
    double xCoordinate;
    double yCoordinate;

    double IN_PER_TICK = 2 * WHEEL_RADIUS_MM * Math.PI / 25.4 / 2000d;
    Pose2d startingPos;
    double[] previousEncoderVals;


    public TwoWheelOdometry(HardwareMap hardwareMap, Pose2d startingPosition, Telemetry telemetry) {
        startingPos = startingPosition;
        previousEncoderVals = new double[2];

        imu = new IMUWrapper(hardwareMap);

        yEncoder = hardwareMap.get(DcMotor.class, "yEncoder");
        yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        xEncoder = hardwareMap.get(DcMotor.class, "xEncoder");
        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        xEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        timer = new Timer();

        resetEncoders();
    }

    long lastXTicks;
    long lastYTicks;
    double yaw;
    double lastYaw;
    double currentTime;
    double previousTime;
    double deltaTime;
    public void update() {
        imu.update();
        yaw = imu.getYaw();
        currentTime = timer.updateTime();

        deltaTime = currentTime - previousTime;

        long currentXTicks = xEncoder.getCurrentPosition();
        long currentYTicks = yEncoder.getCurrentPosition();

        long deltaXTicks = currentXTicks - lastXTicks;
        long deltaYTicks = currentYTicks - lastYTicks;


        double tempYaw = (yaw + lastYaw) / 2;

        double deltaYaw = yaw - lastYaw;
        double deltaXInches = IN_PER_TICK * deltaXTicks;
        double deltaYInches = IN_PER_TICK * deltaYTicks;

        // arcs from center of rotation
        deltaXInches += deltaYaw * IN_PER_MM * CENTER_ODOMETRY_DISTANCE_MM;
        deltaYInches += deltaYaw * IN_PER_MM * VERTICAL_ODOMETRY_DISTANCE_MM;

//        xCoordinate += deltaXInches * Math.cos(tempYaw) - deltaYInches * Math.sin(tempYaw);
//        yCoordinate += deltaXInches * Math.sin(tempYaw) + deltaYInches * Math.cos(tempYaw);

        xCoordinate += deltaXInches;
        yCoordinate += deltaYInches;

        lastXTicks = currentXTicks;
        lastYTicks = currentYTicks;

        lastYaw = yaw;
        previousTime = currentTime;
    }

    public double getXCoordinate() {
        return xCoordinate;
    }

    public double getYCoordinate() {
        return yCoordinate;
    }

    public double getYaw() {
        return imu.getYaw();
    }

    public void resetEncoders(){
        xEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        xEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public List<Double> getWheelVelocities() {
        double deltaLeftTicks = yEncoder.getCurrentPosition() - previousEncoderVals[0];
        double deltaFrontTicks = xEncoder.getCurrentPosition() - previousEncoderVals[1];

        double leftWheelVelocity = IN_PER_TICK * deltaLeftTicks / deltaTime;
        double frontWheelVelocity = IN_PER_TICK * deltaFrontTicks / deltaTime;

        return Arrays.asList(leftWheelVelocity, frontWheelVelocity);
    }

    public double getTrackWidth() {
        return trackDistance;
    }

    public double getWheelBase() {
        return DriveConstants.WHEEL_BASE;
    }

    public double getLateralMultiplier() {
        return LateralMultiplier;
    }


}

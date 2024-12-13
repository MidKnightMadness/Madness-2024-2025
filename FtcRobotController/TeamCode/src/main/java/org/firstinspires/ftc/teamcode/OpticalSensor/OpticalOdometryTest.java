package org.firstinspires.ftc.teamcode.OpticalSensor;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Helper.Pose;

@TeleOp(name = "Optical Odometry")
@Disabled
public class OpticalOdometryTest extends OpMode {

    SparkFunOTOS sensor;
    SparkFunOTOS.Pose2D pose;
    private Object x;

    @Override
    public void init() {
        sensor = hardwareMap.get(SparkFunOTOS.class, "odometry sensor");

        sensor.begin();
        telemetry.addLine("Started");

        sensor.calibrateImu();
        telemetry.addLine("Calibrating");
        sensor.resetTracking();

        telemetry.addLine("Resetting Tracking");

        telemetry.addData("Self Test Completed", sensor.selfTest());
    }

    boolean previousdPadPressed;
    @Override
    public void loop() {
        pose = sensor.getPosition();
        telemetry.addLine("Position");

        telemetry.addData("X Position", pose.x);

        telemetry.addData("Y Position", pose.y);
        telemetry.addData("Heading", pose.h);

        telemetry.addData("Velocity", new Pose(sensor.getVelocity().x, sensor.getVelocity().y, sensor.getVelocity().h).toString());
        telemetry.addData("Acceleration", new Pose(sensor.getAcceleration().x,sensor.getAcceleration().y,sensor.getAcceleration().h).toString());
        telemetry.addData("Angular Scalar", sensor.getAngularScalar());

        if(gamepad1.dpad_left && previousdPadPressed == false){
            sensor.resetTracking();
        }

        previousdPadPressed = gamepad1.dpad_left;

    }
}

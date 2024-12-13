package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.Helper.Timer;

public class TwoWheelOdometry {
    IMUWrapper imu;
    public DcMotor yEncoder;
    public DcMotor xEncoder;
    Timer timer;
    Telemetry telemetry;

    final double WHEEL_RADIUS_MM = 16;

    double xCoordinate;
    double yCoordinate;

    double IN_PER_TICK = 2 * WHEEL_RADIUS_MM * Math.PI / 25.4 / 2000d;

    public TwoWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();

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
    public void update() {
        imu.update();
        yaw = imu.getYaw();

        long currentXTicks = xEncoder.getCurrentPosition();
        long currentYTicks = yEncoder.getCurrentPosition();

        long deltaXTicks = currentXTicks - lastXTicks;
        long deltaYTicks = currentYTicks - lastYTicks;

        double deltaXInches = IN_PER_TICK * deltaXTicks;
        double deltaYInches = IN_PER_TICK * deltaYTicks;

        double tempYaw = (yaw + lastYaw) / 2;
        xCoordinate += deltaXInches * Math.cos(tempYaw) - deltaYInches * Math.sin(tempYaw);
        yCoordinate += deltaXInches * Math.sin(tempYaw) + deltaYInches * Math.cos(tempYaw);

        lastXTicks = currentXTicks;
        lastYTicks = currentYTicks;

        lastYaw = yaw;
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
}

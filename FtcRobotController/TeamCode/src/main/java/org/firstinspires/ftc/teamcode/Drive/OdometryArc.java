package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryArc {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    double[] position;
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor frontEncoder;
    ElapsedTime timer;

    double wheelRadius = 3.429;
    double ticksPerRotation = 8192;
    double robotRadiusToHoriEncoder = 6.3;

    double currentTime;
    double previousTime = 0.0;

    public OdometryArc(HardwareMap hardwareMap, Telemetry telemetry, double[] initial){

        leftEncoder = hardwareMap.get(DcMotor.class, "FL");
        rightEncoder = hardwareMap.get(DcMotor.class, "FR");
        frontEncoder = hardwareMap.get(DcMotor.class, "BR");

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        position = initial;

        timer = new ElapsedTime();
        timer.startTime();
    }

    double velocity[] = new double[2];
    double previousEncoderVals[] = new double[3];

    double deltaField[] = new double[3];
    double updateRate = 1 / 4;

    public void update(){
        currentTime = timer.time();
        double leftEncoderdDist = (leftEncoder.getCurrentPosition() - previousEncoderVals[0]) / ticksPerRotation * (2 * Math.PI * wheelRadius);
        double rightEncoderdDist = (rightEncoder.getCurrentPosition() - previousEncoderVals[1]) / ticksPerRotation * (2 * Math.PI * wheelRadius);

        deltaField[2] = (leftEncoderdDist + rightEncoderdDist) / (2 * robotRadiusToHoriEncoder);//angle

        double distanceTraveled = (leftEncoderdDist + rightEncoderdDist) / 2;
        //modeling it as linear -> because update rate is fast, linear is normally sufficient

        deltaField[0] = distanceTraveled * Math.cos(deltaField[2]);
        deltaField[1] = distanceTraveled * Math.sin(deltaField[2]);

        position[0] = position[0] + deltaField[0];
        position[1] = position[1] + deltaField[1];
        position[2] = position[2] + deltaField[2];


        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();

        velocity[0] = deltaField[0] / (currentTime - previousTime);
        velocity[1] = deltaField[1] / (currentTime - previousTime);

        updateRate = 1 / (currentTime - previousTime);
        previousTime = timer.time();

    }

    public void telemetry(){
        telemetry.addLine("----------------------------------");
        telemetry.addData("DX", deltaField[0]);
        telemetry.addData("DY", deltaField[1]);
        telemetry.addData("DT", deltaField[2]);

        telemetry.addLine("----------------------------------");
        telemetry.addData("X Coordinate", getXCoordinate());
        telemetry.addData("Y Coordinate", getYCoordinate());
        telemetry.addData("Rotation", getRotationDegrees());

        telemetry.addLine("----------------------------------");
        telemetry.addData("Encoder Left Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Encoder Right Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Encoder Front Position", frontEncoder.getCurrentPosition());

        telemetry.addLine("----------------------------------");
        telemetry.addData("Velocity X", getVelocityX());
        telemetry.addData("Velocity Y", getVelocityY());
        telemetry.addData("Elapsed Time", getTotalTime());
        telemetry.addData("Update Rate", getUpdateRate());
    }

    public double getUpdateRate() {
        return 1 / (currentTime - previousTime);
    }

    public double getTotalTime() {
        return currentTime;
    }

    public double getVelocityY() {
        return velocity[1];
    }

    public double getVelocityX() {
        return velocity[0];
    }
    public double getAccelerationX(){
        return velocity[0] / getUpdateRate();
    }

    public double getAccelerationY(){
        return velocity[1] / getUpdateRate();
    }

    public double getRotationDegrees() {
        return position[2] * 180 / Math.PI;
    }

    public double getRotationRadians(){
        return position[2];
    }

    public double getYCoordinate() {
        return position[1];
    }

    public double getXCoordinate() {
        return position[0];
    }


}

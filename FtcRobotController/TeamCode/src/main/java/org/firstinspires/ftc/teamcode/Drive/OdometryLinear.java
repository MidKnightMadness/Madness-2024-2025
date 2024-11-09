package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//3 dead wheel localization

public class OdometryLinear {

    double cmPerTick = (2 * Math.PI * 3.429)/ 8192; //
    double distBetweenHori = 12.2 ;//in
    double distVertEncoders = 0.14;//in
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotor leftEncoder;
    public DcMotor rightEncoder;
    public DcMotor frontEncoder;
    double position[];
    ElapsedTime elapsedTime;
    public OdometryLinear(HardwareMap hardwareMap, Telemetry telemetry, double[] starting){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        this.position = starting;//x, y, theta

        //actual robot
//        leftEncoder = hardwareMap.get(DcMotor.class, "FL");
//        rightEncoder = hardwareMap.get(DcMotor.class, "FR");
//        frontEncoder = hardwareMap.get(DcMotor.class, "BR");

        //test robot
        leftEncoder = hardwareMap.get(DcMotor.class, "BL");
        rightEncoder = hardwareMap.get(DcMotor.class, "FL");
        frontEncoder = hardwareMap.get(DcMotor.class, "FR");

        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }
    double currentTime;
    double previousTime = 0.0;

    double velocity[] = new double[2];
    double previousEncoderVals[] = new double[3];

    double deltaFieldX;
    double deltaFieldY;
    double deltaTheta;

    public void update(){
        double leftEncoderDist = leftEncoder.getCurrentPosition() - previousEncoderVals[0];
        double rightEncoderDist = rightEncoder.getCurrentPosition() - previousEncoderVals[1];
        double frontEncoderDist = frontEncoder.getCurrentPosition() - previousEncoderVals[2];

        double differenceOverHor = (leftEncoderDist - rightEncoderDist)/ distBetweenHori;

        double deltaRobotX = cmPerTick * (leftEncoderDist + rightEncoderDist)/2;
        deltaTheta = cmPerTick * differenceOverHor;

        double deltaRobotY = cmPerTick * (frontEncoderDist - distVertEncoders *  differenceOverHor);

        //change to field coordinates

        deltaFieldX = position[0] + deltaRobotX * Math.cos(deltaTheta) - deltaRobotY * Math.sin(deltaTheta);
        deltaFieldY = position[1] + deltaRobotX * Math.sin(deltaTheta) + deltaRobotY * Math.cos(deltaTheta);

        position[0] = position[0] + deltaFieldX;
        position[1] = position[1] + deltaFieldY;
        position[2] = position[2] + deltaTheta;

        velocity[0] = deltaFieldX / (elapsedTime.time() - previousTime);
        velocity[1] = deltaFieldY / (elapsedTime.time() - previousTime);

        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();
        previousTime = elapsedTime.time();
    }

    public void telemtry(){
        telemetry.addLine("----------------------------------");
        telemetry.addData("DX", deltaFieldX);
        telemetry.addData("DY", deltaFieldY);
        telemetry.addData("DT", deltaTheta);

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
        telemetry.addData("TimeCall", getTime());
    }

    public double getXCoordinate(){
        return position[0];
    }

    public double getTotalTime(){
        return currentTime;
    }

    public double getTime(){
        return currentTime - previousTime;
    }
    public double getUpdateRate(){
        return (1 / (currentTime - previousTime));
    }

    public double getYCoordinate(){
        return position[1];
    }

    public double getRotationDegrees(){
        return position[3] * 180 / Math.PI;
    }

    public double getRotationRadians(){
        return position[3];
    }

    public double getVelocityX(){
        return velocity[0];
    }
    public double getVelocityY(){
        return velocity[1];
    }



}

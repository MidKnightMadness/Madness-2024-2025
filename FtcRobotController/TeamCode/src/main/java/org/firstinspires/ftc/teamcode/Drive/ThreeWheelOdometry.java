package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.PathingRR.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PathingRR.Util.Encoder;

import java.util.Arrays;
import java.util.List;

//3 dead wheel localization

public class ThreeWheelOdometry {
    double LateralMultiplier = 1;


    double inPerEncoderTick = 1 / 25.4 * 32 * Math.PI / 2000d;
    double trackDistance = 12.1;//in //11 for old chassis
    double distVertEncoders = 3;//in

    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;
    double position[] = new double[3];
    double startingPosition[] = new double[3];
    IMUWrapper imu;
    ElapsedTime elapsedTime;
    public ThreeWheelOdometry(HardwareMap hardwareMap, Telemetry telemetry, double[] starting){
        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        for(int i = 0; i < 3; i++){
            startingPosition[i] = starting[i];
            position[i] = starting[i];
            rotation = position[2];
        };//x, y, theta


        //actual robot
//        leftEncoder = hardwareMap.get(DcMotor.class, "FL");
//        rightEncoder = hardwareMap.get(DcMotor.class, "FR");
//        frontEncoder = hardwareMap.get(DcMotor.class, "BR");

        //test robot
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));

        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();

        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }
    public ThreeWheelOdometry(HardwareMap hardwareMap, double[] starting){
        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        for(int i = 0; i < 3; i++){
            startingPosition[i] = starting[i];
            position[i] = starting[i];
            rotation = position[2];
        };//x, y, theta


        //actual robot
//        leftEncoder = hardwareMap.get(DcMotor.class, "FL");
//        rightEncoder = hardwareMap.get(DcMotor.class, "FR");
//        frontEncoder = hardwareMap.get(DcMotor.class, "BR");

        //test robot
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FR"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BL"));

        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();

        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }
    public void resetEncoders(){
       leftEncoder.resetEncoders();
       rightEncoder.resetEncoders();
       frontEncoder.resetEncoders();;
    }
    double currentTime;
    double previousTime = 0.0;

    double velocity[] = new double[3];
    double previousEncoderVals[] = new double[3];

    double deltaRobotX;
    double deltaRobotY;
    double deltaFieldX;
    double deltaFieldY;
    double deltaTheta;
    double thetaMultiplier = 1.1;//madness chassis 2022

    double deltaTime;
    double yaw;

    double previousTheta;
    double rotation;
    public void update(){
        imu.update();
        yaw = normalizeAngleRadians(imu.getYaw());
        rotation = yaw;

        double deltaLeftTicks = leftEncoder.getCurrentPosition() - previousEncoderVals[0];
        double deltaRightTicks = rightEncoder.getCurrentPosition() - previousEncoderVals[1];
        double deltaFrontTicks = frontEncoder.getCurrentPosition() - previousEncoderVals[2];

        //left and right encoders are negative when moving forward, horizontal encoder positive when rotating clockwise

        deltaRobotX = inPerEncoderTick * (deltaLeftTicks + deltaRightTicks) / 2;
        deltaTheta =  -inPerEncoderTick * (deltaLeftTicks - deltaRightTicks) / trackDistance * thetaMultiplier;
        //ignore delta theta
        deltaRobotY = -inPerEncoderTick * (deltaFrontTicks - distVertEncoders * (deltaRightTicks + deltaLeftTicks) / trackDistance);

        //change to field coordinates

        //rotation = normalizeAngleRadians(normalizeAngleRadians(deltaTheta/2) + normalizeAngleRadians(rotation));
        //still have to normalize rotation

        deltaFieldX = -(deltaRobotX * Math.cos(rotation) - deltaRobotY * Math.sin(rotation));
        deltaFieldY = deltaRobotX * Math.sin(rotation) + deltaRobotY * Math.cos(rotation);

        position[0] = position[0] + deltaFieldX;
        position[1] = position[1] + deltaFieldY;
        position[2] = normalizeAngleRadians(position[2] + normalizeAngleRadians(deltaTheta));


        deltaTime = elapsedTime.time() - previousTime;

        velocity[0] = deltaFieldX / deltaTime;
        velocity[1] = deltaFieldY / deltaTime;
        velocity[2] = deltaTheta / deltaTime;

        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();

        //rotation = normalizeAngleRadians(normalizeAngleRadians(deltaTheta/2) + normalizeAngleRadians(rotation));
        previousTime = elapsedTime.time();
    }

    public void telemetry(){

        telemetry.addLine("Odometry Linear");
        telemetry.addData("IMUYaw", yaw);
        telemetry.addData("DeltaTheta(Encoder)", deltaTheta);
        telemetry.addData("CombinedIMU&Encoder", rotation);
        telemetry.addLine("----------------------------------");
        telemetry.addData("Encoder Left Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Encoder Right Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Encoder Front Position", frontEncoder.getCurrentPosition());

        telemetry.addLine("----------------------------------");

        telemetry.addData("DeltaLeftTicks", leftEncoder.getCurrentPosition() - previousEncoderVals[0]);
        telemetry.addData("DeltaRightTicks", rightEncoder.getCurrentPosition() - previousEncoderVals[1]);
        telemetry.addData("DeltaFrontTicks", frontEncoder.getCurrentPosition() - previousEncoderVals[2]);

        telemetry.addLine("----------------------------------");

        telemetry.addData("DX", deltaFieldX);
        telemetry.addData("DY", deltaFieldY);
        telemetry.addData("DT", deltaTheta);

        telemetry.addLine("----------------------------------");
        telemetry.addData("X Coordinate", getXCoordinate());
        telemetry.addData("Y Coordinate", getYCoordinate());
        telemetry.addData("Rotation", getRotationDegrees());

        telemetry.addLine("----------------------------------");
        telemetry.addData("Delta RobotX", deltaRobotX);
        telemetry.addData("Delta RobotY", deltaRobotY);

        telemetry.addLine("----------------------------------");
        telemetry.addData("Velocity X", getVelocityX());
        telemetry.addData("Velocity Y", getVelocityY());
        telemetry.addData("Velocity Angular", getVelocityTheta());
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
        return normalizeAngleDegrees(position[2] * 180 / Math.PI);
    }

    public double normalizeAngleDegrees(double rotation){
        return((rotation + 360) % 360);
    }//0 to 2 pi
    public double normalizeAngleRadians(double rotation){
        return((rotation + 2 * Math.PI) % (2 * Math.PI));
    }
    public double getRotationRadians(){
        return position[2];
    }

    public double getVelocityX(){
        return velocity[0];
    }
    public double getVelocityY(){
        return velocity[1];
    }
    public double getVelocityTheta(){ return velocity[2]; }


    public List<Double> getWheelVelocities() {
        double deltaLeftTicks = leftEncoder.getCurrentPosition() - previousEncoderVals[0];
        double deltaRightTicks = rightEncoder.getCurrentPosition() - previousEncoderVals[1];
        double deltaFrontTicks = frontEncoder.getCurrentPosition() - previousEncoderVals[2];

        double leftWheelVelocity = inPerEncoderTick * deltaLeftTicks / deltaTime;
        double rightWheelVelocity = inPerEncoderTick * deltaRightTicks / deltaTime;
        double frontWheelVelocity = inPerEncoderTick * deltaFrontTicks / deltaTime;

        return Arrays.asList(leftWheelVelocity, rightWheelVelocity, frontWheelVelocity);

    }

    public double getTrackWidth() {
        return DriveConstants.TRACK_WIDTH;
    }

    public double getWheelBase() {
        return DriveConstants.WHEEL_BASE;
    }

    public double getLateralMultiplier() {
        return LateralMultiplier;
    }
}
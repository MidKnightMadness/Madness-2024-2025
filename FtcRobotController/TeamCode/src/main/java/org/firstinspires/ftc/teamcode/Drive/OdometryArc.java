package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OdometryArc {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    double[] position = new double[3];
    DcMotor leftEncoder;
    DcMotor rightEncoder;
    DcMotor frontEncoder;
    ElapsedTime timer;

    double wheelRadius = 16;
    double ticksPerRotation = 2000;
    double trackDistance = 12.1;

    double distHorEncoders = 0.14;

    double currentTime;
    double previousTime = 0.0;
    double rotation = 0;
    double[] startingPosition = new double[3];

    public void resetEncoders(){
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public OdometryArc(HardwareMap hardwareMap, Telemetry telemetry, double[] initial){
        leftEncoder = hardwareMap.get(DcMotor.class, "FL");
        rightEncoder = hardwareMap.get(DcMotor.class, "FR");
        frontEncoder = hardwareMap.get(DcMotor.class, "BR");

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        for(int i = 0; i < 3; i++){
            startingPosition[i] = initial[i];
            position[i] = initial[i];
            rotation = position[2];
        }

        timer = new ElapsedTime();
        timer.startTime();
    }

    boolean currentState;
    double velocity[] = new double[2];
    double previousEncoderVals[] = new double[3];

    double deltaField[] = new double[3];
    double deltaRobot[] = new double[3];
    double updateRate = 1 / 4;
    double previousTheta = 0;

    public void update(){
        currentTime = timer.time();
        double deltaLeftTicks = leftEncoder.getCurrentPosition() - previousEncoderVals[0];
        double deltaRightTicks = rightEncoder.getCurrentPosition() - previousEncoderVals[1];
        double deltaFrontTicks = frontEncoder.getCurrentPosition() - previousEncoderVals[2];

        double leftEncoderDist = (deltaLeftTicks) / ticksPerRotation * (2 * Math.PI * wheelRadius) / 25.4;
        double rightEncoderDist = (deltaRightTicks) / ticksPerRotation * (2 * Math.PI * wheelRadius) / 25.4;
        double frontEncoderDist = (deltaFrontTicks) / ticksPerRotation * (2 * Math.PI * wheelRadius) / 25.4;

        double deltaTheta = -(leftEncoderDist - rightEncoderDist) / (2 * trackDistance);//theta
        //when both encoders moving foward become more negative

        //average the rotation val
        rotation = normalizeAngleRadians(position[2] + normalizeAngleRadians(deltaTheta));

        if(Math.abs(deltaTheta) < 10e-6){//if strafing or straight perfectly- no change to theta
            currentState = false;
            deltaRobot[0] = frontEncoderDist;
            deltaRobot[1] = (leftEncoderDist + rightEncoderDist) / 2;
        }
        else{//some angle change
            currentState = true;
            deltaRobot[0] = 2 * (frontEncoderDist / deltaTheta + distHorEncoders) * Math.sin(deltaTheta / 2);
            deltaRobot[1] = 2 * (leftEncoderDist / deltaTheta + trackDistance / 2) * Math.sin(deltaTheta / 2);
        }

        //change to global coords
        deltaField[0] = deltaRobot[0] * Math.cos(rotation) - deltaRobot[1] * Math.sin(rotation);
        deltaField[1] = deltaRobot[0] * Math.sin(rotation) + deltaRobot[1] * Math.cos(rotation);


        position[0] = position[0] + deltaField[0];
        position[1] = position[1] + deltaField[1];
        position[2] = rotation;


        previousEncoderVals[0] = leftEncoder.getCurrentPosition();
        previousEncoderVals[1] = rightEncoder.getCurrentPosition();
        previousEncoderVals[2] = frontEncoder.getCurrentPosition();

        velocity[0] = deltaField[0] / (currentTime - previousTime);
        velocity[1] = deltaField[1] / (currentTime - previousTime);

        updateRate = 1 / (currentTime - previousTime);
        previousTime = timer.time();

    }

    public void telemetry(){
        telemetry.addLine("Odometry Arc");
        telemetry.addLine("----------------------------------");
        telemetry.addData("Angle change", currentState);
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


    public double normalizeAngleDegrees(double rotation){
        return(rotation % 360);
    }
    public double normalizeAngleRadians(double rotation){
        return(rotation % (2 * Math.PI));
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

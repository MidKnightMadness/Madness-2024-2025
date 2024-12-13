package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Timer;

public class MecanumDrive {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    //RPM Constants Madness 2024 Robot
//    double FLmulti = ((double) 1);
//    double FRmulti = ((double) 1);
//    double BLmulti = ((double) 1);
//    double BRmulti = ((double) 1);
    

    //RPM Constants Old Chassis
    double FLmulti =  1;
    double FRmulti =  1;
    double BLmulti = 1;
    double BRmulti = 1;

    Telemetry telemetry;
    private double[] motorInputs;
    private double[] motorFinalInputs = new double[4];
    private double[] motorVelocity = new double[4];
    double previousX = 0.0;
    double previousY = 0.0;
    Timer timer;


    public void resetEncoders(){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        FR.setDirection(DcMotorSimple.Direction.REVERSE);
//        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        //Old chassis :
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;


        timer = new Timer();
        resetEncoders();
        motorInputs = new double[4];
    }

    public void velocityDrive(double x, double y, double rotation, double maxVelocity){
        double FLPower = y + x + rotation;
        double FRPower = y - x - rotation;
        double BLPower = y - x + rotation;
        double BRPower = y + x - rotation;


        motorInputs = new double[]{FLPower * FLmulti, FRPower * FRmulti, BLPower * BLmulti, BRPower * BRmulti};
        double maxPower = 0;

        for(int i = 0; i < 4; i++){
            maxPower = Math.max(maxPower, motorInputs[i]);
        }

        if(maxPower > 1) {
            for (int i = 0; i < 4; i++) {
                motorInputs[i] = motorInputs[i] / maxPower;
            }
        }


        setVelocity(motorInputs[0], motorInputs[1], motorInputs[2], motorInputs[3], maxVelocity);
    }

    public void normalDrive(double x, double y, double rotation, double power){
        double FLPower = y + x + rotation;
        double FRPower = y - x - rotation;
        double BLPower = y - x + rotation;
        double BRPower = y + x - rotation;


        motorInputs = new double[]{FLPower * FLmulti, FRPower * FRmulti, BLPower * BLmulti, BRPower * BRmulti};
        double maxPower = 0;



        for(int i = 0; i < 4; i++){
            maxPower = Math.max(maxPower, motorInputs[i]);
        }

        if(maxPower > 1) {
            for (int i = 0; i < 4; i++) {
                motorInputs[i] = (double) motorInputs[i] / maxPower;
            }
        }

        setPowers(motorInputs[0], motorInputs[1], motorInputs[2], motorInputs[3], power);
        timer.updateTime();
        for(int i = 0; i < 4; i++) {
            motorVelocity[i] = motorInputs[i] / timer.getDeltaTime();
        }


    }

    double lowPassConstantPrevious = 0.1;


    public void FieldOrientedDrive(double x, double y, double rotation, double angle, double power){

        //Keep lowPass value realtively low for teleOp to respond quicker, higher for auto for smoothness
        double lowPassX = lowPassConstantPrevious * previousX + (1-lowPassConstantPrevious) * x;
        double lowPassY = lowPassConstantPrevious * previousY + (1-lowPassConstantPrevious) * y;


        //rotate x axis to y axis -> and convert
        double newX = lowPassX * Math.cos(angle - Math.PI/2) + lowPassY * Math.sin(angle - Math.PI/2);
        double newY = -lowPassX * Math.sin(angle - Math.PI/2) + lowPassY * Math.cos(angle - Math.PI/2);

        double FLPower = newY + newX + rotation;
        double FRPower = newY - newX - rotation;
        double BLPower = newY - newX + rotation;
        double BRPower = newY + newX - rotation;

        double maxPower = Math.max(FLPower, Math.max(FRPower, Math.max(BLPower, BRPower)));

        if(maxPower > 1) {
            FLPower /= maxPower;
            FRPower /= maxPower;
            BLPower /= maxPower;
            BRPower /= maxPower;
        }


        setPowers(FLPower, FRPower, BLPower, BRPower, power);
    }


    public void updateTelemetry(){
        telemetry.addLine("----Motor Powers ------------");
        telemetry.addData("FL",  motorFinalInputs[0]);
        telemetry.addData("FR",  motorFinalInputs[1]);
        telemetry.addData("BL",  motorFinalInputs[2]);
        telemetry.addData("BR",  motorFinalInputs[3]);
    }

    public double[] getVelocity(){
        return motorVelocity;
    }
    public void setPowers(double FLP, double FRP, double BLP, double BRP, double power) {

        motorFinalInputs[0] = FLP * power;
        motorFinalInputs[1] = FRP * power;
        motorFinalInputs[2] = BLP * power;
        motorFinalInputs[3] = BRP * power;

        FL.setPower(FLP * power);
        FR.setPower(FRP * power);
        BL.setPower(BLP * power);
        BR.setPower(BRP * power);
    }

    private void setVelocity(double FLP, double FRP, double BLP, double BRP, double maxVelocity) {

        motorFinalInputs[0] = FLP * maxVelocity;
        motorFinalInputs[1] = FRP * maxVelocity;
        motorFinalInputs[2] = BLP * maxVelocity;
        motorFinalInputs[3] = BRP * maxVelocity;

        FL.setVelocity(FLP * maxVelocity);
        FR.setVelocity(FRP * maxVelocity);
        BL.setVelocity(BLP * maxVelocity);
        BR.setVelocity(BRP * maxVelocity);
    }

    public double[] getMotorInputs(){
        return motorFinalInputs;
    }


}

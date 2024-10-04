package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDrive {
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;
    Telemetry telemetry;
    private double[] motorInputs;
    private double[] motorFinalInputs;
    double previousX = 0.0;
    double previousY = 0.0;
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

        motorInputs = new double[4];
    }

    public void normalDrive(double x, double y, double rotation, double power){
        double FLPower = y + x + rotation;
        double FRPower = y - x - rotation;
        double BLPower = y - x + rotation;
        double BRPower = y + x - rotation;


        motorInputs = new double[]{FLPower, FRPower, BLPower, BRPower};
        double maxPower = 0;

        for(int i = 0; i < 4; i++){
            maxPower = Math.max(maxPower, motorInputs[i]);
        }

        for(int i = 0; i < 4; i++){
            motorInputs[i] = motorInputs[i] / maxPower;
        }

        setPowers(motorInputs[0], motorInputs[1], motorInputs[2], motorInputs[3], power);
    }

    double lowPassConstantPrevious = 0.1;


    public void FieldOrientedDrive(double x, double y, double rotation, double power){

        //Keep lowPass value realtively low for teleOp to respond quicker, higher for auto for smoothness
        double lowPassX = lowPassConstantPrevious * previousX + (1-lowPassConstantPrevious) * x;
        double lowPassY = lowPassConstantPrevious * previousY + (1-lowPassConstantPrevious) * y;


        //rotate x axis to y axis -> and convert
        double newX = lowPassX * Math.cos(rotation - Math.PI/2) + lowPassY * Math.sin(rotation - Math.PI/2);
        double newY = -lowPassX * Math.sin(rotation - Math.PI/2) + lowPassY * Math.cos(rotation - Math.PI/2);

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
    private void setPowers(double FLP, double FRP, double BLP, double BRP, double power) {

        motorFinalInputs[0] = FLP * power;
        motorFinalInputs[1] = FRP * power;
        motorFinalInputs[2] = BLP * power;
        motorFinalInputs[3] = BRP * power;

        FL.setPower(FLP * power);
        FR.setPower(FRP * power);
        BL.setPower(BLP * power);
        BR.setPower(BRP * power);

    }

}

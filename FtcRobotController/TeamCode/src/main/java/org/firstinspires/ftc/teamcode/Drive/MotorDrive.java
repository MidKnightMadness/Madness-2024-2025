package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp(name = "motor test")
public class MotorDrive extends OpMode {

    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;
    double FLMultiplier = 0.5;
    double FRMultiplier = 0.5;
    double BLMultiplier = 0.5;
    double BRMultiplier = 0.5;

    double previousFL = 0;
    double previousFR = 0;
    double previousBL = 0;
    double previousBR = 0;

    Timer timer;

    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FR = hardwareMap.get(DcMotor.class, "FR");
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR = hardwareMap.get(DcMotor.class, "BR");
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL = hardwareMap.get(DcMotor.class, "BL");
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();

    }

    double RPMfl;
    double RPMfr;
    double RPMbl;
    double RPMbr;
    @Override
    public void loop() {
        telemetry.addData("Time", timer.updateTime());
        timer.updateTime();
        telemetry.addData("Delta Time", timer.getDeltaTime());

        FL.setPower(1);
        FR.setPower(1);
        BL.setPower(1);
        BR.setPower(1);


        if(gamepad1.left_bumper){
           RPMfl = ( FL.getCurrentPosition() / timer.getDeltaTime());
           RPMfr = FR.getCurrentPosition() / timer.updateTime();
           RPMbl = BL.getCurrentPosition() / timer.updateTime();
           RPMbr = BR.getCurrentPosition() / timer.updateTime();
        }

        telemetry.addData("FL Pos", FL.getCurrentPosition());
        telemetry.addData("FR Pos", FR.getCurrentPosition());
        telemetry.addData("BL Pos", BL.getCurrentPosition());
        telemetry.addData("BR Pos", BR.getCurrentPosition());
//
//
        telemetry.addData("RPM FL", RPMfl);
        telemetry.addData("RPM FR", RPMfr);
        telemetry.addData("RPM BL", RPMbl);
        telemetry.addData("RPM BR", RPMbr);




        //test
        previousBR = BR.getCurrentPosition();
        previousBL = BL.getCurrentPosition();
        previousFL = FL.getCurrentPosition();
        previousFR = FR.getCurrentPosition();

        telemetry.update();
    }
}

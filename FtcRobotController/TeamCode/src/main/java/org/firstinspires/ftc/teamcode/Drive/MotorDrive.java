package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp(name = "motor test")
public class MotorDrive extends OpMode {

    public DcMotor FR;
    public DcMotor FL;
    public DcMotor BR;
    public DcMotor BL;
    Timer timer;

    @Override
    public void init() {
        FR = hardwareMap.get(DcMotor.class, "FL");
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL = hardwareMap.get(DcMotor.class, "FR");
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        BR = hardwareMap.get(DcMotor.class, "BR");
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        BL = hardwareMap.get(DcMotor.class, "BL");
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new Timer();

    }

    @Override
    public void loop() {
        telemetry.addData("Time", timer.updateTime());
        FL.setPower(1);
        FR.setPower(1);

//        BL.setPower(1);
//        BR.setPower(1);

        telemetry.addData("FL Pos", FL.getCurrentPosition());
        telemetry.addData("FR Pos", FR.getCurrentPosition());

        telemetry.addData("RPM - FL", (double) FL.getCurrentPosition() / timer.updateTime());
        telemetry.addData("RPM - FR", (double) FR.getCurrentPosition() / timer.updateTime());

        //test
    }
}

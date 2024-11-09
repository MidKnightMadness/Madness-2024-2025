package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class OdoDrive {
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;

    DcMotor leftOdo;
    DcMotor rightOdo;
    DcMotor frontOdo;
    IMU imu;


    public OdoDrive(HardwareMap hardwareMap){
        //initialize motors
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontOdo = hardwareMap.get(DcMotor.class, "BL");
        rightOdo = hardwareMap.get(DcMotor.class, "FL");
        leftOdo = hardwareMap.get(DcMotor.class, "BR");

        frontOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }



}

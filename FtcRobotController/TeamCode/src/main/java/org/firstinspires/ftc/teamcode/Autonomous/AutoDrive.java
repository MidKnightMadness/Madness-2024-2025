package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drive.OdometryLinear;
import org.firstinspires.ftc.teamcode.Drive.PIDBasic;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp()
public class AutoDrive extends OpMode {
    Timer timer;
    MecanumDrive mecanumDrive;
    OdometryLinear odometryLinear;
    PIDBasic pid;
    IMU imu;

    double[] pidPoints = new double[]{};
    double[] startingPosition = new double[]{7.5, 7.5, Math.PI / 2};
    @Override
    public void init() {
        timer = new Timer();
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometryLinear = new OdometryLinear(hardwareMap, telemetry, startingPosition);
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

    }

    @Override
    public void loop() {


    }
}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp(name = "Straight Test")
public class StraightAutonomousTest extends OpMode {
    DeadReckoningDrive deadReckoningDrive;
    MecanumDrive mecanumDrive;
    Timer timer;
    IMU imu;
    @Override
    public void init() {
        timer = new Timer();
        timer = new Timer();
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//        odometryLinear = new OdometryLinear(hardwareMap, telemetry, startingPosition);
        deadReckoningDrive = new DeadReckoningDrive(hardwareMap, telemetry);

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    @Override
    public void loop() {
        deadReckoningDrive.driveForward(10, telemetry);
    }
}

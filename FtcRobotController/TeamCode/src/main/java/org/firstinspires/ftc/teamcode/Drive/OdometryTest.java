package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Drive.OdometryArc;
import org.firstinspires.ftc.teamcode.Drive.OdometryLinear;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp(name = "Odometry Test")
public class OdometryTest extends OpMode{

    MecanumDrive mecanumDrive;
    OdometryLinear linearOdo;
    ButtonToggle left1Bumper, right1Bumper;


    IMU imu;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        left1Bumper = new ButtonToggle();
        right1Bumper = new ButtonToggle();

        linearOdo = new OdometryLinear(hardwareMap, telemetry, new double[]{7.75,7.25, Math.toRadians(90)});

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    boolean isUsingFieldOriented;
    double drivePower = 0.8;
    @Override
    public void loop() {

        if(left1Bumper.update(gamepad1.left_bumper)){
            isUsingFieldOriented = !isUsingFieldOriented;
            gamepad1.rumble(100);
        }


        if(right1Bumper.update(gamepad1.right_bumper)){
            if(drivePower == 0.8){
                drivePower = 0.5;
                gamepad1.rumble(300);
            }
            else if(drivePower == 0.5){
                drivePower = 0.3;
                gamepad1.rumble(400);
            }
            else{
                drivePower = 0.8;
                gamepad1.rumble(500);
            }
        }

        if(gamepad1.dpad_up){
            mecanumDrive.resetEncoders();
        }

        linearOdo.update();



        linearOdo.telemetry();

        if(isUsingFieldOriented){
            mecanumDrive.velocityDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 1000);
            telemetry.addLine("Velocity Drive");
        }
        else {
            mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
            telemetry.addLine("Normal Drive");
        }


        telemetry.addData("IMU Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}

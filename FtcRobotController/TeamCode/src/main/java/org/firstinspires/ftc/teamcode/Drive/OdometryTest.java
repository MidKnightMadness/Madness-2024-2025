package org.firstinspires.ftc.teamcode.Drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Drive.OdometryArc;
import org.firstinspires.ftc.teamcode.Drive.OdometryLinear;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;


@Config
@TeleOp(name = "Odometry Test")
public class OdometryTest extends OpMode{

    MecanumDrive mecanumDrive;
    OdometryLinear linearOdo;
    OdometryArc odometryArc;
    ButtonToggle left1Bumper, right1Bumper;


    IMU imu;
    TelemetryPacket telemetryPacket;
    FtcDashboard ftcDashboard;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        left1Bumper = new ButtonToggle();
        right1Bumper = new ButtonToggle();

        linearOdo = new OdometryLinear(hardwareMap, telemetry, new double[]{7.75,7.25, Math.toRadians(90)});

        odometryArc = new OdometryArc(hardwareMap, telemetry, new double[]{7.25, 7.25, Math.toRadians(90)});
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        telemetryPacket = new TelemetryPacket();
        ftcDashboard = FtcDashboard.getInstance();
    }

    boolean isUsingFieldOriented;
    double drivePower = 0.8;
    @Override
    public void loop() {

        telemetry.addData("GamepadY", gamepad1.left_stick_y);

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
        odometryArc.update();




        linearOdo.telemetry();

        odometryArc.telemetry();
        telemetry.addLine("Voltage");

        telemetry.addData("VoltageFL", mecanumDrive.FL.getConnectionInfo());
        telemetry.addData("VoltageFR", mecanumDrive.FR.getConnectionInfo());
        telemetry.addData("VoltageBL", mecanumDrive.BL.getConnectionInfo());
        telemetry.addData("VoltageBR", mecanumDrive.BR.getConnectionInfo());

        telemetryPacket.put("Xval", linearOdo.getXCoordinate());
        telemetryPacket.put("Yval", linearOdo.getYCoordinate());

        telemetryPacket.put("Rotation", linearOdo.getRotationDegrees());

        telemetryPacket.put("FLPow", mecanumDrive.getMotorInputs()[0]);
        telemetryPacket.put("FRPow", mecanumDrive.getMotorInputs()[1]);
        telemetryPacket.put("BLPow", mecanumDrive.getMotorInputs()[2]);
        telemetryPacket.put("BRPow", mecanumDrive.getMotorInputs()[3]);

        telemetryPacket.put("FLVel", mecanumDrive.getVelocity()[0]);
        telemetryPacket.put("FRVel", mecanumDrive.getVelocity()[1]);
        telemetryPacket.put("BLVel", mecanumDrive.getVelocity()[2]);
        telemetryPacket.put("BRVel", mecanumDrive.getVelocity()[3]);

        ftcDashboard.sendTelemetryPacket(telemetryPacket);


        mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
        telemetry.addLine("Normal Drive");




        mecanumDrive.updateTelemetry();
        telemetry.addData("IMU Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.Drive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@TeleOp(name = "IMU")
public class IMUTest extends OpMode {

    IMU imu;
    double initialYaw;

    @Override
    public void init() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        initialYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    @Override
    public void loop() {

        if(gamepad1.left_bumper){
            imu.resetYaw();
        }

        telemetry.addData("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
        telemetry.addData("Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        telemetry.addData("Angle/Yaw", normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));

        telemetry.update();
    }

    double normalizeAngle(double angle) {
        return mod((angle + 180), 360) - 180;
    }

    double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }
}

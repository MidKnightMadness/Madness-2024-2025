package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drive.TwoWheelOdometry;
import org.firstinspires.ftc.teamcode.Helper.Timer;

public class ErrorDrive {
    MecanumDrive mecanumDrive;
    Telemetry telemetry;
    Timer timer;

    public ErrorDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        this.telemetry = telemetry;
        timer = new Timer();
    }

    public boolean drive(double currentX, double currentY, double currentYaw, double targetX, double targetY, double targetYaw, double velocity, double pTranslation, double pRotation) {
        double xError = targetX - currentX;
        double yError = targetY - currentY;
        double yawError = targetYaw - currentYaw;

        // clamp values
        double xPower = Math.signum(xError) * Math.abs(xError * pTranslation);
        double yPower = Math.signum(yError) * Math.abs(yError * pTranslation);
        double rPower = Math.signum(yawError) * Math.abs(yawError * pRotation);
        mecanumDrive.velocityDrive(xPower, yPower, rPower, velocity);

        return Math.sqrt(xError * xError + yError * yError) < 0.5;
    }

    public void driveToPosition(TwoWheelOdometry twoWheelOdometry, double targetX, double targetY, double targetYaw, double velocity, double pTranslation, double pRotation, double maxTime) {
        double startTime = timer.updateTime();
        while (drive(twoWheelOdometry.getXCoordinate(), twoWheelOdometry.getYCoordinate(), twoWheelOdometry.getYaw(), targetX, targetY, targetYaw, velocity, pTranslation, pRotation)) {
            if (timer.updateTime() - startTime > maxTime) break;

            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Yaw", targetYaw);
            telemetry.update();
        }
    }
}

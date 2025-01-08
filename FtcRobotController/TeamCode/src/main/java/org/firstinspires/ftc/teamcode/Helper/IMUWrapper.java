/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Helper;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUWrapper
{
    IMU imu;
    Telemetry telemetry;
    Timer timer;

    double yaw = 0;

    public IMUWrapper(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(IMU.class, "imu");
        this.telemetry = telemetry;

        timer = new Timer();
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    double normalizeAngle(double angle) {
        return mod((angle + 180), 360) - 180;
    }

    double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }

    final double biasCalibrationTime = 0d;

    double[] imuBiases = new double[6];
    double yawVelBias;
    double lastYawVel;

    public void calibrateBiases() {
        telemetry.addLine("Ensure the robot is not moving");
        telemetry.addLine("Calibrating");
        telemetry.update();

        double startTime = timer.updateTime();
        double sumIMUYawVel = 0;
        int numSamples = 0;

        while (timer.updateTime() < startTime + biasCalibrationTime) {
            sumIMUYawVel += imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            numSamples += 1;
        }

        yawVelBias = sumIMUYawVel / numSamples;
        telemetry.addData("Yaw Velocity Bias", yawVelBias);
        telemetry.update();
    }

    public void update() {
        timer.updateTime();
        double yawVel = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate - yawVelBias;
        double deltaYaw = timer.getDeltaTime() * (yawVel + lastYawVel);

        yaw += deltaYaw;
    }

    public double getYawVelBias() {
        return yawVelBias;
    }

    public double getYaw() {
        return yaw;
    }

    public double getUncorrectedYaw() {
        return normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }


//    public void logData() {
//        telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
//        // Retrieve Rotational Angles and Velocities
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
//
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
//        telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
//        telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
//        telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
//        telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
//        telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
//        telemetry.update();
//    }

}

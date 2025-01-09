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
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IMUWrapper
{
    IMU imuControlHub;
    BNO055IMU imuExpansionHub;
    Timer timer;

    double yaw = 0;

    public IMUWrapper(HardwareMap hardwareMap) {
        BNO055IMU.Parameters expansionIMUParameters = new BNO055IMU.Parameters();
        expansionIMUParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        expansionIMUParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        expansionIMUParameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample OpMode
        expansionIMUParameters.loggingEnabled      = true;
        expansionIMUParameters.loggingTag          = "IMU";
        expansionIMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imuControlHub = hardwareMap.get(IMU.class, "imuControl");
        imuExpansionHub = hardwareMap.get(BNO055IMU.class, "imuExpansion");

        timer = new Timer();
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imuControlHub.initialize(new IMU.Parameters(orientationOnRobot));
        imuExpansionHub.initialize(expansionIMUParameters);
    }

    public static double normalizeAngle(double angle) {
        return mod((angle + Math.PI), 2 * Math.PI) - Math.PI;
    }

    public static double mod(double num, double divisor) {
        return num - Math.floor(num / divisor) * divisor;
    }
    double lastYawVel;

    public void update() {
        timer.updateTime();

        double controlHubVel = imuControlHub.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        double expansionHubVel = imuExpansionHub.getAngularVelocity().zRotationRate;
        double yawVel = (controlHubVel + expansionHubVel) / 2;
        double deltaYaw = timer.getDeltaTime() * (yawVel + lastYawVel) / 2;

        yaw += deltaYaw;

        lastYawVel = yawVel;
    }


    public double getYaw() {
        return getExpansionHubYaw();
    }

    public double getControlHubYaw() {
        return imuControlHub.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public double getYawVelocity() {
        return imuExpansionHub.getAngularVelocity().zRotationRate;
    }
    public double getExpansionHubYaw() {
        return imuExpansionHub.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


}

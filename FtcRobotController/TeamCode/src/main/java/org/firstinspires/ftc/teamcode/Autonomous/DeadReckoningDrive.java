package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Helper.Timer;




public class DeadReckoningDrive {
    DcMotor fr;
    DcMotor fl;
    DcMotor bl;
    DcMotor br;

    DcMotor leftOdo;
    DcMotor rightOdo;
    DcMotor frontOdo;
    IMU imu;

    double cmPerTick = (2 * Math.PI * 3.429)/ 8192; //
    double distBetweenHori = 12.2 ;//in
    double distVertEncoders = 0.14;//in

    MecanumDrive mecanumDrive;

    double lateralDisplacement;
    double verticalDisplacement;

    Timer timer;

    public DeadReckoningDrive(HardwareMap hardwareMap, Telemetry telemetry){
        //initialize motors
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");


        mecanumDrive = new MecanumDrive(hardwareMap,telemetry);

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

        timer = new Timer();
    }

    public void updateDisplacement(){
        lateralDisplacement = frontOdo.getCurrentPosition();
        verticalDisplacement = Math.abs(leftOdo.getCurrentPosition() - leftOdo.getCurrentPosition()) / 2;
    }

    //don't need theta
    public void resetDisplacement(){
        lateralDisplacement = 0;
        verticalDisplacement = 0;
    }


    public void resetEncoders(){
        leftOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontOdo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForwardDistance(double distance, double maxPower, double seconds, boolean timed, Telemetry telemetry) {
        double minPower = 0.15;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop && timer.updateTime() - startTime < seconds) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }


            error = distance - verticalDisplacement;
            double direction = Math.signum(error);

            double power = minPower + ((error > 5)? (maxPower - minPower) : maxPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addData("Time elapsed", timer.updateTime() - startTime);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * direction, power * direction, power * direction, power * direction);
        }

        setPowers(0, 0, 0, 0);
    }
//    public void moveForwardDistance(double distance) {
//        moveForwardDistance(distance, 0.5);
//    }
//
//    public void moveForwardDistance(double distance, double maxPower, double targetAngle){moveForwardDistance(distance, maxPower, targetAngle, 10, true);}

    public void moveForwardDistance(double distance, double maxPower, double targetAngle, double time, boolean timed, Telemetry telemetry) {
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.16;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        int updates = 0;
        double rotationCorrection = 0;
        while (Math.abs(error) > errorToStop && timer.updateTime() - startTime < time) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            if(updates++ % 10 == 0) {
                currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
                rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                if (rotationCorrection < 0.5) {
                    rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                } else {
                    rotationCorrection = 0;
                }
            }

            error = distance - verticalDisplacement;
            double direction = Math.signum(error);

            double power = minPower + ((error > 5)? (maxPower - minPower) : maxPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection), power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }

    public void moveForwardDistance(double distance, double maxPower, double targetAngle, double seconds, Telemetry telemetry) {
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.16;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        int updates = 0;
        double rotationCorrection = 0;
        while (Math.abs(error) > errorToStop && timer.updateTime() - startTime < seconds) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            if(updates++ % 10 == 0) {
                currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0) ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
                rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                if (rotationCorrection < 0.5) {
                    rotationCorrection = targetAngle * Math.PI / 180d - currentAngleCorrected;
                } else {
                    rotationCorrection = 0;
                }
            }

            error = distance - verticalDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

            telemetry.addData("Error", error);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection), power * (direction + rotationCorrectionConstant * rotationCorrection), power * (direction - rotationCorrectionConstant * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }



    public void moveRightDistance(double distance, Telemetry telemetry) {
        double minPower = 0.275;
        double maxPower = 0.8;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            error = distance - lateralDisplacement;
            double direction = Math.signum(error);

            double power = minPower + ((error > 5)? maxPower : maxPower - minPower) * Math.abs(error / distance);

            telemetry.clear();
            telemetry.addData("Error", error);
            telemetry.addData("Lateral displacement", lateralDisplacement );
            telemetry.addData("Top ticks", frontOdo.getCurrentPosition());
            telemetry.addData("Distance", distance);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * direction, power * -direction, power * -direction, power * direction);
        }

        setPowers(0, 0, 0, 0);
    }

    private final double rotationCorrectionConstant = 0.0;
    private final double rotationCorrectionConstantForRotation = 0.15;

    public void moveRightDistance(double distance, double targetAngle, Telemetry telemetry) { // Angle in radians
        double currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
        double minPower = 0.275;
        double maxPower = (Math.abs(distance) > 5)? 0.8 : 0.5;

        resetDisplacement();

        double startTime = timer.updateTime();
        double currentTime = startTime;

        double error = distance;
        double errorToStop = 0.1;
        while (Math.abs(error) > errorToStop) {
            if (currentTime - startTime > 6) {
                errorToStop += 0.05;
            }
            updateDisplacement();

            // Makes sure angle is between 0 and 360˚
            currentAngleCorrected = (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) > 0)? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 2 * Math.PI;
            double rotationCorrection = targetAngle * Math.PI / 180 - currentAngleCorrected;

            error = distance - lateralDisplacement;
            double direction = Math.signum(error);

            double power = minPower + ((error > 5)? maxPower : maxPower - minPower) * Math.abs(error / distance);

            telemetry.clear();
            telemetry.addData("Error", error);
            telemetry.addData("Angle error", rotationCorrection);
            telemetry.addData("Rotation Correction Power", rotationCorrectionConstantForRotation * rotationCorrection);
            telemetry.addData("Power", power * direction);
            telemetry.addLine("-------");

            telemetry.update();

            setPowers(power * (direction + rotationCorrectionConstantForRotation * rotationCorrection), power * -(direction - rotationCorrectionConstantForRotation * rotationCorrection), power * -(direction + rotationCorrectionConstantForRotation * rotationCorrection), power * (direction - rotationCorrectionConstantForRotation * rotationCorrection));
        }

        setPowers(0, 0, 0, 0);
    }

    double backDropToWallTolerance = 10d;



    void driveForward(double distance, Telemetry telemetry) {
        double minPower = 0.225;
        double maxPower = 0.5;

        resetDisplacement();
        double error;

        do {
            updateDisplacement();

            error = distance - verticalDisplacement;
            double direction = Math.signum(error);

            double power = minPower + (maxPower - minPower) * Math.abs(error / 16d);

            power = Math.min(power, maxPower) * direction;

            setPowers(power, power, power, power);

            telemetry.clear();
            telemetry.addLine(String.format("Power: %.3f", power));
            telemetry.addLine(String.format("Error: %.2f", error));
            telemetry.addLine(String.format("Forward displacement %.3f", verticalDisplacement));
            telemetry.update();

        } while (Math.abs(error) > 0.1);

        setPowers(0, 0, 0, 0);
    }

//    void setPowersForTimeSmoothed(double seconds, double fl, double fr, double bl, double br) {
//        double startTime = timer.updateTime();
//
//        // run for time
//        while (timer.updateTime() - startTime < seconds) {
//            setPowersSmoothed(fl, fr, bl, br, 1/50d);
////            telemetryMotorVelocities();
//            timer.updateTime();
//        }
//
//        while (Math.abs(FL.getPower()) > 0.1) {
//            setPowersSmoothed(0, 0, 0, 0, 1/50);
//        }
//
//        setPowers(0, 0, 0, 0);
//    }

    void setMotorPowersForTime(double seconds, double fl, double fr, double bl, double br, Telemetry telemetry) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.updateTime() - startTime < seconds) {
            setPowers(fl, fr, bl, br);
            telemetry.clear();
            telemetry.addData("Time elapsed", timer.updateTime() - startTime);
            telemetry.update();
        }

        setPowers(0, 0, 0, 0);
    }

    void setMotorVelocitiesForTime(double seconds, double x, double y, double rot, double maxVelocity, Telemetry telemetry) {
        double startTime = timer.updateTime();

        // run for time
        while (timer.updateTime() - startTime < seconds) {
            mecanumDrive.velocityDrive(x, y, rot, maxVelocity);
            telemetry.clear();
            telemetry.addData("Time elapsed", timer.updateTime() - startTime);
            telemetry.update();
        }

        setPowers(0, 0, 0, 0);
    }



    void setPowers(double FLPower, double FRPower, double BLPower, double BRPower){
        fl.setPower(FLPower);
        fr.setPower(FRPower);
        bl.setPower(BLPower);
        br.setPower(BRPower);

    }
}

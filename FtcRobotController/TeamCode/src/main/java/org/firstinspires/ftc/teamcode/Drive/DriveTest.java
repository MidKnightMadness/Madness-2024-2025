package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp(name  = "Drive Robot")
public class DriveTest extends OpMode {
//    PIDBasic PIDDrive;
    //    OdometryLinear odometry;
    MecanumDrive mecanumDrive;
    public double[] targetStates = {40, 60, 20};
    Timer timer;
    IMU imu;
//    ButtonToggle left1Bumper, right1Bumper;
//    boolean isUsingFieldOriented;

    @Override
    public void init() {
//        left1Bumper = new ButtonToggle();
//        right1Bumper = new ButtonToggle();
        timer = new Timer();
//        odometry = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0, Math.PI/2});
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        telemetry.update();

//        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
//        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward
//
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//
//        imu.resetYaw();
    }

    double[] pidOutputs = new double[3];
    double drivePower = 0.8;

    @Override
    public void loop() {
//        odometry.update();

//
//        //POINT to POINT PID
//        PIDDrive = new PIDBasic(odometry, targetStates[0], targetStates[1], targetStates[2], telemetry);
//        pidOutputs = PIDDrive.updatePID();
//        mecanumDrive.FieldOrientedDrive(pidOutputs[0], pidOutputs[1], pidOutputs[2], drivePower);
//        telemetry.addLine("Point to Point PID");

        //Driver Field Oriented


//            odometry.update();

//        if (left1Bumper.update(gamepad1.left_bumper)) {
//            isUsingFieldOriented = !isUsingFieldOriented;
//            gamepad1.rumble(100);
//        }


//        if(right1Bumper.update(gamepad1.right_bumper)){
//            if(drivePower == 0.8){
//                drivePower = 0.5;
//                gamepad1.rumble(300);
//            }
//            else if(drivePower == 0.5){
//                drivePower = 0.3;
//                gamepad1.rumble(400);
//            }
//            else{
//                drivePower = 0.8;
//                gamepad1.rumble(500);
//            }
//        }
        //Robot oriented

//
//        if(isUsingFieldOriented){
//            mecanumDrive.velocityDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 1000);
//            telemetry.addLine("Velocity Drive");
//////            odometry.update();
//////            mecanumDrive.veloictyTest(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
//////            telemetry.addLine("Velocity set");
////
////            telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
////            telemetry.addData("Gamepad 1 Left stick y", gamepad1.left_stick_y);
////            telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);
////            telemetry.addData("Drive Power", power);
////
////            telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
////            telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
////            telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
////            telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
////            double rotation = ((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - 180) % 360 + 180) + Math.PI / 2d;
////            telemetry.addData("Rotation", rotation);
////
////            mecanumDrive.FieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, rotation, power);
//            telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
//            telemetry.addData("Gamepad 1 Left stick y", gamepad1.left_stick_y);
//            telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);
//
//            telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
//            telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
//            telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
//            telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
//            mecanumDrive.updateTelemetry();
//        }
//        else{
//            odometry.update();
        telemetry.addLine("Robot Oriented");
        mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);

        telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Gamepad 1 Left stick y", -gamepad1.left_stick_y);
        telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);
        telemetry.addData("Drive Power", drivePower);


        telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
        telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
        telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
        telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
        mecanumDrive.updateTelemetry();
    }
//        telemetry.addData("Robot X", odometry.getXCoordinate());
//        telemetry.addData("Robot Y", odometry.getYCoordinate());
//        telemetry.addData("Robot Theta", odometry.getRotationDegrees());
//
//        telemetry.addData("Robot X Velocity", odometry.getVelocityX());
//        telemetry.addData("Robot Y Velocity", odometry.getVelocityY());
//
//        telemetry.addData("Time", timer.updateTime());
//
//        telemetry.update();
//
//    }
}


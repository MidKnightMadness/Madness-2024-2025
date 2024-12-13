package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.EndEffector.ServoPositions;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp(name  = "Drive Robot")
public class DriveTest extends OpMode implements ServoPositions {
    TwoWheelOdometry odometry;
    MecanumDrive mecanumDrive;

    Timer timer;
    IMUWrapper imu;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    CRServo leftServo;
    CRServo rightServo;


    Servo wristServo;
    Servo clawServo;

    @Override
    public void init() {
        timer = new Timer();

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wristServo = hardwareMap.get(Servo.class, "leftWristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new TwoWheelOdometry(hardwareMap, telemetry);

        leftServo = hardwareMap.get(CRServo.class, "Horizontal Slides Left");
        rightServo = hardwareMap.get(CRServo.class, "Horizontal Slides Right");

        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();
        telemetry.update();
    }

    double drivePower = 0.8;

    @Override
    public void loop() {
        telemetry.addLine("Robot Oriented");
        mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);

        int verticalDirection = gamepad1.right_bumper ? 1 : -1;
        runVerticalWithPosition(gamepad1.right_trigger, verticalDirection);
//        rightMotor.setPower(gamepad1.right_trigger * verticalDirection);
//        leftMotor.setPower(gamepad1.right_trigger * verticalDirection);

        int horizontalDirection = gamepad1.left_bumper ? 1 : -1;
        leftServo.setPower(gamepad1.left_trigger * horizontalDirection);
        rightServo.setPower(gamepad1.left_trigger * horizontalDirection);

        // rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.2, 0.4, 0.5, 0.05));

        imu.update();
        odometry.update();

        telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
        telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
        telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
        telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());

        telemetry.addData("Right motor pos", rightMotor.getCurrentPosition());
        telemetry.addData("Left motor pos", leftMotor.getCurrentPosition());

        telemetry.addData("X", odometry.getXCoordinate());
        telemetry.addData("Y", odometry.getYCoordinate());
        telemetry.addData("Yaw", odometry.getYaw() * 180 / Math.PI);

        mecanumDrive.updateTelemetry();
    }

    double changeTicks;
    void runVerticalWithPosition(double power, int direction) {
        changeTicks += (power * direction);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + (int) changeTicks);
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + (int) changeTicks);
    }

    void runVerticalWithPowers(double power, int direction) {
        rightMotor.setPower(power * direction);
        leftMotor.setPower(power * direction);
    }
}


package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    Servo leftWristServo;
    Servo clawServo;

    ButtonToggle a1;
    ButtonToggle y1;
    ButtonToggle x1;
    ButtonToggle b1;


    @Override
    public void init() {
        timer = new Timer();

        a1 = new ButtonToggle();
        b1 = new ButtonToggle();
        x1 = new ButtonToggle();
        y1 = new ButtonToggle();

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftWristServo = hardwareMap.get(Servo.class, "leftWristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new TwoWheelOdometry(hardwareMap, new Pose2d(0,0,Math.PI/2), telemetry);

        leftServo = hardwareMap.get(CRServo.class, "leftHorizontal");
//        rightServo = hardwareMap.get(CRServo.class, "Horizontal Slides Right");
        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();
        telemetry.update();

//        clawServo.setPosition(ServoPositions.grabNeutral);
        leftWristServo.setPosition(ServoPositions.armServoLeftNeutral);
    }

    double drivePower = 0.8;

    @Override
    public void loop() {
        telemetry.addLine("Robot Oriented");
        mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);

        int verticalDirection = gamepad1.right_bumper ? 1 : -1;
        rightMotor.setPower(gamepad1.right_trigger * verticalDirection);
        leftMotor.setPower(gamepad1.right_trigger * verticalDirection);


        int horizontalDirection = gamepad1.left_bumper ? 1 : -1;
//        leftServo.setPosition(0.5 + 0.5 * gamepad1.left_trigger * horizontalDirection);
        leftServo.setPower(gamepad1.left_trigger * horizontalDirection);

        telemetry.addData("Left servo", gamepad1.left_trigger * horizontalDirection);
//        rightServo.setPower(gamepad1.left_trigger * horizontalDirection);

        // rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(0.2, 0.4, 0.5, 0.05));

        if(a1.update(gamepad1.b)){
            clawServo.setPosition(ServoPositions.grabClosed);
        }
        if(x1.update(gamepad1.x)){
            clawServo.setPosition(ServoPositions.grabOpen);
        }

        if (gamepad1.dpad_up) {
            clawServo.setPosition(ServoPositions.grabNeutral);
        }

        if(y1.update(gamepad1.y)){
            leftWristServo.setPosition(ServoPositions.armServoLeftSample);
        }
        if(a1.update(gamepad1.a)){
            leftWristServo.setPosition(ServoPositions.armServoLeftNeutral);
        }

        imu.update();
        odometry.update();

        telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
        telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
        telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
        telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());

        telemetry.addData("Right motor velocity", rightMotor.getVelocity());
        telemetry.addData("Left motor velocity", leftMotor.getVelocity());

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


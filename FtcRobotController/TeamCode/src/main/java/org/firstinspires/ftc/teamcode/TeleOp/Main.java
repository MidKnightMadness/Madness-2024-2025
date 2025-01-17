package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.EndEffector.ServoPositions;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Slides.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Slides.VerticalSlides;

import org.firstinspires.ftc.teamcode.Helper.Timer;


@TeleOp(group= "Game", name = "Driver Controlled TeleOp")
public class Main extends OpMode implements ServoPositions {
//    boolean gameTime = false; //player two can take over drive
    MecanumDrive mecanumDrive;
//    OdometryLinear odometry;
    VerticalSlides verticalSlides;
    HorizontalSlides horizontalSlides;
    Servo armServoLeft, armServoRight, clawServo;
    ColorSensor clawColorSensor;
    ColorSensorWrapper colorSensorWrapper;
    ButtonToggle left1Bumper, right1Bumper, A1, X1, Y1;
    IMU imu;
    boolean isUsingFieldOriented;
    Timer timer;

    @Override
    public void init() {
      mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//      odometry = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0,0});
      verticalSlides = new VerticalSlides(hardwareMap);
      armServoLeft = hardwareMap.get(Servo.class, "Arm Servo Left");
      armServoRight = hardwareMap.get(Servo.class, "Arm Servo Right");

      clawServo = hardwareMap.get(Servo.class, "Claw Grabber");
      clawColorSensor = hardwareMap.get(ColorSensor.class, "Claw Color Sensor");
      colorSensorWrapper = new ColorSensorWrapper(clawColorSensor, 2);

      left1Bumper = new ButtonToggle();
      right1Bumper = new ButtonToggle();
      timer = new Timer();

      init_IMU();

      telemetry.addLine("Initialized");
    }

    double power = 0.8;
//    double rotationResetConstant = 0.0;

    public void handleDriverControls() {

        if(left1Bumper.update(gamepad1.left_bumper)){
            isUsingFieldOriented = !isUsingFieldOriented;
            gamepad1.rumble(100);
        }

        if(right1Bumper.update(gamepad1.right_bumper)){
            if(power == 0.8){
                power = 0.5;
                gamepad1.rumble(300);
            }
            else if(power == 0.5){
                power = 0.3;
                gamepad1.rumble(400);
            }
            else{
                power = 0.8;
                gamepad1.rumble(500);
            }
        }


        if(isUsingFieldOriented){
            mecanumDrive.velocityDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 1000);
            telemetry.addLine("Velocity Drive - Power is not used");
////            odometry.update();
////            mecanumDrive.veloictyTest(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, drivePower);
////            telemetry.addLine("Velocity set");
//
//            telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
//            telemetry.addData("Gamepad 1 Left stick y", gamepad1.left_stick_y);
//            telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);
//            telemetry.addData("Drive Power", power);
//
//            telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
//            telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
//            telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
//            telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
//            double rotation = ((imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - 180) % 360 + 180) + Math.PI / 2d;
//            telemetry.addData("Rotation", rotation);
//
//            mecanumDrive.FieldOrientedDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, rotation, power);
            telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 Left stick y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);

            telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
            telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
            telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
            telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
            mecanumDrive.updateTelemetry();
        }
        //Robot oriented

        else {
//            odometry.update();
            mecanumDrive.normalDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, power);

            telemetry.addLine("Robot Oriented");

            telemetry.addData("Gampead 1 Left stick x", gamepad1.left_stick_x);
            telemetry.addData("Gamepad 1 Left stick y", gamepad1.left_stick_y);
            telemetry.addData("Gamepad 1 Right Stick x", gamepad1.right_stick_x);
            telemetry.addData("Drive Power", power);


            telemetry.addData("FR Velocity", mecanumDrive.FR.getVelocity());
            telemetry.addData("FL Velocity", mecanumDrive.FL.getVelocity());
            telemetry.addData("BR Velocity", mecanumDrive.BR.getVelocity());
            telemetry.addData("BL Velocity", mecanumDrive.BL.getVelocity());
            mecanumDrive.updateTelemetry();

        }
        //reset imu at 90 degrees angle(facing backstage) when a is clicked
        if (gamepad1.a) { // Temporary for field oriented drive, may come up with auto align functionality
            imu.resetYaw();
//            rotationResetConstant = Math.PI / 2; // Assumes resetting at 90˚ from starting position, aka facing backstage side
        }

//        telemetry.addData("Driver mode", isUsingFieldOriented ? "Field Oriented" : "Normal");
        telemetry.addData("Driver speed", power == 0.8 ? "High" : power == 0.5 ? "Medium" : "Low");
    }

    @Override
    public void start() {
        clawServo.setPosition(ServoPositions.grabNeutral);
        armServoLeft.setPosition(ServoPositions.armServoLeftSample);
        armServoRight.setPosition(ServoPositions.armServoRightSample);
        //face down
        //set vertical slides to neutral position
        //set horizontal Slides to neutral position

    }
    @Override
    public void loop() {
        handleDriverControls();
        handleManipulatorControls();
    }

    boolean clawClosed;
    public void handleManipulatorControls() {
        //helpful presets
        if(A1.update(gamepad1.a)){
            armServoRight.setPosition(ServoPositions.armServoRightSpecimen);

//            verticalSlides.extendToDistance(armSpecimenHeight);
        }

        if(X1.update(gamepad1.x)){
//            armServoLeft.setPosition(ServoPositions.armServoLeftSample);
//            armServoRight.setPosition(ServoPositions.armServoRightSample);
//            verticalSlides.extendToDistance(armOuttakeHeight);
        }

        if(Y1.update(gamepad1.y)){
            armServoLeft.setPosition(ServoPositions.armServoLeftSample);
            armServoRight.setPosition(ServoPositions.armServoRightSample);
//            verticalSlides.extendToDistance(armSampleHeight);
        }

//        //slides control
//        if (gamepad2.left_stick_x != 0 || gamepad2.left_stick_y != 0) {
//            horizontalSlides.extend(gamepad2.left_stick_x);
//            verticalSlides.verticalSlidesExtend(gamepad2.left_stick_y);
//        }

        if(gamepad2.right_bumper){
            if(!clawClosed){
                clawServo.setPosition(ServoPositions.grabClosed);
            }
            else{
                clawServo.setPosition(ServoPositions.grabOpen);
            }
        }

        //if grabbing sample, it will detect color
        if(clawServo.getPosition() == ServoPositions.grabClosed) {
            colorSensorWrapper.update();
            RGBColor rgbColor = colorSensorWrapper.getValue();

            SampleColors.Colors detected = ColorClassifier.classify(rgbColor);

            telemetry.addData("Detected Color", detected);
        }

//        if (gamepad2.left_bumper && !gameTime) { //disable this for actual game
//            mecanumDrive.normalDrive(power, -gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
//        }

    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }


}

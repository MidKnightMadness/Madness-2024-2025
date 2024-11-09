package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Drive.OdometryLinear;
import org.firstinspires.ftc.teamcode.EndEffector.ServoPositions;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Slides.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Slides.VerticalSlides;

import org.firstinspires.ftc.teamcode.Helper.Timer;

@Config
@TeleOp(group= "Game", name = "Driver Controlled TeleOp")
public class OnePlayerMain extends OpMode implements ServoPositions {
   // boolean gameTime = false; //player two can take over drive
    MecanumDrive mecanumDrive;
    OdometryLinear odometry;
    VerticalSlides verticalSlides;
    HorizontalSlides horizontalSlides;
    Servo armServoLeft, armServoRight, clawServo;
    ColorSensor clawColorSensor;
    ColorSensorWrapper colorSensorWrapper;
    ButtonToggle left1Bumper, right1Bumper, A1, B1, X1, Y1, dpadDOWN1, dpadDOWN2, dpadDOWN3, dpadDOWN4;
    IMU imu;
    boolean isUsingFieldOriented;
    Timer timer;


    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
//        odometry = new OdometryLinear(hardwareMap, telemetry, new double[]{0,0,0});
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
    double rotationResetConstant = 0.0;

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


        if (!isUsingFieldOriented) {
            //if gamepad 1 left bumper not turned on, or if turned off, toggle normal drive controlled by driver one x, y movements on sticks
            mecanumDrive.normalDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, power);
        }
        else {
            //can use gamepad 1 left bumper to toggle field oriented drive controlled by driver one x, y movements on sticks
            mecanumDrive.FieldOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d + rotationResetConstant, power);
        }




        //reset imu at 90 degrees angle(facing backstage) when a is clicked
        if (gamepad1.a) { // Temporary for field oriented drive, may come up with auto align functionality
            imu.resetYaw();
            rotationResetConstant = Math.PI / 2; // Assumes resetting at 90˚ from starting position, aka facing backstage side
        }

        telemetry.addData("Driver mode", isUsingFieldOriented ? "Field Oriented" : "Normal");
        telemetry.addData("Driver speed", power == 0.8 ? "High" : power == 0.5 ? "Medium" : "Low");
    }

    @Override
    public void start() {
        clawServo.setPosition(ServoPositions.grabNeutral);
//        armServoLeft.setPosition(ServoPositions.armServoLeftOpen);
//        armServoRight.setPosition(ServoPositions.armServoRightOpen);
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
           // armServoLeft
        }

        if(X1.update(gamepad1.x)){
           // armServo.setPosition(ServoPositions.armOutakeHeight);
        }

        if(Y1.update(gamepad1.y)){
          //  armServo.setPosition(ServoPositions.armSampleHeight);
        }


        if(gamepad1.right_bumper){
            if(!clawClosed){
                clawServo.setPosition(ServoPositions.grabClosed);
            }
            else{
                clawServo.setPosition(ServoPositions.grabOpen);
            }
        }


        /*if (gamepad1.left_bumper && !gameTime) { //disable this for actual game
            mecanumDrive.normalDrive(power, -gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
        }*/

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

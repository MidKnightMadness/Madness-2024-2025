package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.ServoPositions;
import org.firstinspires.ftc.teamcode.Drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.Localization.AprilTagLocalizerTwo;
import org.firstinspires.ftc.teamcode.Testing.ColorSensor.OutakeColorSensors;
import org.firstinspires.ftc.teamcode.Utility.ButtonToggle;
import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;
import org.firstinspires.ftc.teamcode.Utility.Timer;
//import org.firstinspires.ftc.teamcode.Utility.ServoSmooth;

@TeleOp(group= "aGame", name = "Driver Controlled TeleOp")
public class Main extends OpMode implements ServoPositions {
    MecanumDrive mecanumDrive;
    DcMotor verticalMotorR, verticalMotorL;
    ButtonToggle g1A, g1RightBump, g1LeftBump;
    public DcMotorEx motorRight, motorLeft;
    Servo armServo, clawServo, horizontalServoR, horizontalServoL;
    ButtonToggle g2Y, g2A, g2LeftBump, g2RightBump, g2X, g2Share;
   // double [] intakeStackHeights = {intakeLowest, intakeStackOfTwo, intakeStackOfThree, intakeStackOfFour, intakeStackOfFive};
    int intakePreset = 0; // Index from intake heights presets
    int rightSideStartingPosition = 0;
    IMU imu;
   // ModernRoboticsI2cRangeSensor rangeSensor;
  //  WebcamName webcamName;
    //AprilTagLocalizerTwo localizer;

   // Servo launcherServo;
    boolean isUsingFieldOriented;
   // ServoSmooth boxServoController;
    double rotationResetConstant = 0;
    Timer timer;
    double intakeServoPosition;

    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(hardwareMap, telemetry);
        verticalMotorR = hardwareMap.get(DcMotor.class, "verticalMotorR");
      verticalMotorL = hardwareMap.get(DcMotor.class, "verticalMotorL");
        armServo = hardwareMap.get(Servo.class, "armServo");
      clawServo = hardwareMap.get(Servo.class, "clawServo");
      horizontalServoR = hardwareMap.get(Servo.class, "horizontalServoR");
      horizontalServoL = hardwareMap.get(Servo.class, "horizontalServoL");
      
        timer = new Timer();

                init_IMU();
//        outakeColorSensors = new OutakeColorSensors(hardwareMap, telemetry);
       // rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Front Distance Sensor");
       // localizer = new AprilTagLocalizerTwo("Webcam 2", hardwareMap, telemetry, 0, 0);

        //rightElbowServo = hardwareMap.get(Servo.class, "Right elbow servo");
        //rightElbowServo.setPosition(elbowServoIn);

        g2Share = new ButtonToggle();
        g2Y = new ButtonToggle();
        g2A = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        g1A = new ButtonToggle();
        g1RightBump = new ButtonToggle();
        g2X = new ButtonToggle();
        g2LeftBump = new ButtonToggle();
        g1LeftBump = new ButtonToggle();
        /*rightIntakeServo = hardwareMap.get(Servo.class, "Right intake servo");
        leftIntakeServo = hardwareMap.get(Servo.class, "Left intake servo");
        rightWristServo = hardwareMap.get(Servo.class, "Right wrist servo");
*/
     /*   boxServo = hardwareMap.get(Servo.class, "Center box servo");
        boxServoController = new ServoSmooth(boxServo);
*/
        motorLeft = hardwareMap.get(DcMotorEx.class, "Left outtake motor");
        motorRight = hardwareMap.get(DcMotorEx.class, "Right outtake motor");

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSideStartingPosition = motorRight.getCurrentPosition();
        telemetry.addLine("Initialized");

        startingPositions [0] = motorLeft.getCurrentPosition();
        startingPositions [1] = motorRight.getCurrentPosition();
    }

    double power = 1;
    public void handleDriverControls() {
//        telemetry.addData("Left encoder", mecanumDrive.FL.getCurrentPosition());
//        telemetry.addData("Right encoder", mecanumDrive.FR.getCurrentPosition());
//        telemetry.addData("Center encoder", mecanumDrive.BR.getCurrentPosition());
        if (g1RightBump.update(gamepad1.right_bumper)) {//toggle power to 0.35 or max for normal drive by clicking gamepad 1's right bumper
            gamepad1.rumble(300);
            if (power == 1) {
                power = 0.35;
            }
            else {
                power = 1;
            }
        }
        if (g1LeftBump.update(gamepad1.left_bumper)) {
            isUsingFieldOriented = !isUsingFieldOriented;
            gamepad1.rumble(100);
        }

        if (!gamepad1.left_bumper) {
            if (!isUsingFieldOriented) {
                //if gamepad 1 left bumper not turned on, or if turned off, toggle normal drive controlled by driver one x, y movements on sticks
                mecanumDrive.normalDrive(power, -gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }
            else {
                //can use gamepad 1 left bumper to toggle field oriented drive controlled by driver one x, y movements on sticks
                mecanumDrive.FieldOrientedDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x,
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2d + rotationResetConstant, // We start facing left
                        telemetry);
            }
        }



        //reset imu at 90 degrees angle(facing backstage) when a is clicked
        if (gamepad1.a) { // Temporary for field oriented drive, may come up with auto align functionality
            imu.resetYaw();
            rotationResetConstant = Math.PI / 2; // Assumes resetting at 90˚ from starting position, aka facing backstage side
        }

        telemetry.addData("Driver mode", isUsingFieldOriented ? "Field Oriented" : "Normal");
        telemetry.addData("Driver speed", power == 1 ? "High" : "Low");
        telemetry.addData("Left side position", motorLeft.getCurrentPosition());
        telemetry.addData("Right side position", motorRight.getCurrentPosition());
    }

    @Override
    public void start() {
//        while (!boxServoController.setServoPosition(boxServoLeft, 1, telemetry)) {}
//        while (!boxServoController.setServoPosition(boxServoRight, 1, telemetry)) {}
//        while (!boxServoController.setServoPosition(boxServoRight, 1, telemetry)) {}

    }
    @Override
    public void loop() {
        handleDriverControls();
        handleManipulatorControls();

        telemetry();
    }

    double wristPos = wristServoIn;
    double elbowPos = elbowServoIn;
    public void handleManipulatorControls() {
        handleIntakeControls();
        handleOuttakeControls();

        if (gamepad2.left_bumper) {
            mecanumDrive.normalDrive(power, -gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
        }
    

//        if (g2Share.update(gamepad2.share)) {
//            elbowPos = elbowPos == elbowServoIn ? elbowServoVertical : elbowServoIn;
//            rightElbowServo.setPosition(elbowPos);
//        }



        // launcher
      /*  if(gamepad2.dpad_up && gamepad2.y){
            launcherServo.setPosition(launcherOpen);
            telemetry.addLine("Toggled launching Drone");
        } else {
            launcherServo.setPosition(launcherClosed);
        }*/

    }

    Timer intakeTimer = new Timer(); // For allowing better control of intake height
    double intakeAdjustmentStartTime = 0.0;

    void handleIntakeControls() {
        double intakeDirection = gamepad2.a ? 0.4 : -1;

      
      
//        double intakeStartTime = 0;
//        if(gamepad2.left_trigger != 0) {
//            if(!lastLeftTriggerPressed){
//                intakeStartTime = timer.getTime();
//            }
//            if(timer.getTime() - intakeStartTime < 0.25) { // Added a .25 second delay before it actually gets powered
        intakeMotor.setPower(gamepad2.left_trigger * intakeDirection * 0.95);
//            rightIntakeServo.setPosition(intakeLowest);
//            lastLeftTriggerPressed = true;
//        }else{
//            intakeMotor.setPower(0);
//            rightIntakeServo.setPosition(intakeDefault);
//            lastLeftTriggerPressed = false;
//        }
//
//        if(gamepad2.dpad_left && intakePreset < 4){
//            if(timer.updateTime() - intakeAdjustmentStartTime > 0.075){ // .25 second delay minimum for changing position
//                intakeAdjustmentStartTime = timer.updateTime();
//                intakePreset++;
//            }else{
//                timer.updateTime();
//            }
//        }else if(gamepad2.dpad_right && intakePreset > 0){
//            if(timer.updateTime() - intakeAdjustmentStartTime > 0.075){ // .25 second delay minimum for changing position
//                intakeAdjustmentStartTime = timer.updateTime();
//                intakePreset--;
//            }else{
//                timer.updateTime();
//            }
//        }
//        rightIntakeServo.setPosition(intakeStackHeights [intakePreset]);
//        telemetry.addLine("\n\n\n===========================================\nIntakePosition:" + intakePreset + "\n===========================================\n\n\n");


        if (gamepad2.y) {
            rightIntakeServo.setPosition(intakeLowest + (intakeStackOfFive - intakeLowest) * -gamepad2.left_stick_y);;
        }
        else {
            rightIntakeServo.setPosition(intakeLowest);
        }
    }

    // For setting motor bounds and allowing automatic servo movement
    double [] mainExtensionConstants = {0.1, 0.1}; // For both sides to follow based on distance to target; left, right
    int [] slidesBounds = {-2629, 2974};
    int [] startingPositions = {0, 0};
    double inPerTickLeftSlide = -21.5 / 2629d;
    double inPerTickRightSlide = 21.5 / 2974d;

    void handleOuttakeControls() {
        double slidesDirection = gamepad2.a ? -1 : 1;
        motorLeft.setPower(slidesDirection * gamepad2.right_trigger);
        motorRight.setPower(slidesDirection * gamepad2.right_trigger);

        if (this.gamepad2.right_bumper) {
            if (gamepad2.a) {
                boxServoController.setServoPosition(boxServoNeutral, boxServoRight, 0.4, telemetry);  // right
            } else {
                boxServoController.setServoPosition(boxServoNeutral, boxServoLeft, 0.4, telemetry); // left
            }
        } else {
            boxServo.setPosition(boxServoNeutral);  // center
        }

        if(gamepad2.b){
            rightWristServo.setPosition(wristServoFlat);
        }

        if (g2X.update(gamepad2.x)) {
            if (wristPos == wristServoIn) { wristPos = wristServoOut; }
            else { wristPos = wristServoIn; }
            rightWristServo.setPosition(wristPos);
        }

        if (gamepad2.b) {
            rightWristServo.setPosition(wristServoFlat);
            wristPos = wristServoFlat;
        }
//        outakeColorSensors.updateTelemetry();
    }

    void telemetry() {

    }

    void init_IMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;  // logo facing up
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;   // usb facing forward

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();
    }

    double backDropAligned = 3.5d; // CM
    double rotationCorrectionConstant = 0.05;
  /*  public double alignToBoardContinuous(){
        if(rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned > 0.5){
            mecanumDrive.normalDrive(1, 0.0, -(rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned) * 0.05, rotationCorrectionConstant * (Math.PI - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - Math.PI / 2d -  rotationResetConstant));
        }

        telemetry.addData("Distance to board", rangeSensor.getDistance(DistanceUnit.CM));

        return rangeSensor.getDistance(DistanceUnit.CM) - backDropAligned;
    }*/

 /*   public boolean alignToLaunchPositionContinuous(){ // Uses april tags, has contingency to test whether or not  april tags are visible
        return false;
    }*/
}

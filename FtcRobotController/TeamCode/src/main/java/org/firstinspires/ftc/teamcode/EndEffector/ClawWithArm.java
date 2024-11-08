package org.firstinspires.ftc.teamcode.EndEffector;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;


@Config
@TeleOp(name = "ClawWithArm")
public class ClawWithArm extends OpMode {
    Servo grabServo;
    Servo armServoLeft;
    Servo armServoRight;
    ColorSensor colorSensor;

    ColorSensorWrapper colorSensorWrapper;
    Timer timer;
    public static double grabOpen = 0.4;
    public static double grabClosed = 0.635;
    public static double grabNeutral = 0.4;

    //need fixing
    SampleColors.Colors detected = SampleColors.Colors.NONE;
      public double[] LEFTBounds = new double[]{0.3, 0.7};//closed, sample, specimen
      public double[] RIGHTBounds = new double[]{0.65, 0.25};
    public ButtonToggle LeftBumper, rightBumper;

    @Override
    public void init() {
        LeftBumper = new ButtonToggle();
        rightBumper = new ButtonToggle();
        grabServo = hardwareMap.get(Servo.class, "claw grabber");
        armServoLeft = hardwareMap.get(Servo.class, "arm servo left");
        armServoRight = hardwareMap.get(Servo.class, "arm servo right");

        timer = new Timer();
        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, 2);

       grabServo.setPosition(grabNeutral);
       // armServoLeft.setPosition(armRightNeutral);
        armServoLeft.setPosition(LEFTBounds[0]);
        armServoRight.setPosition(RIGHTBounds[0]);

    }

    boolean previousDpadL;
    boolean previousDpadR;
    boolean previousDpadU;
    boolean previousDpadD;
    //double targetPos = 0.5;

    boolean previousGamepadLB;
    boolean previousGamepadRB;
    boolean closeRight;
    boolean closeLeft;
    double change = 0.015;

    @Override
    public void loop() {

        logServoPos();
        telemetry.addData("Current Time", timer.updateTime());
        telemetry.addData("Update Rate", 1 /timer.getDeltaTime());

//        colorSensorWrapper.update();
//        RGBColor rgbColor = colorSensorWrapper.getValue();
//
//        detected = ColorClassifier.classify(rgbColor);
//        telemetry.addData("Color Classification", detected);
//        telemetry.addData("RGBA", rgbColor.toString());


        telemetry.addData("Left Stick" , gamepad1.left_stick_x);
        telemetry.addData("Right Stick", gamepad1.right_stick_x);
        telemetry.update();

        if(gamepad1.dpad_left && previousDpadL == false ){ //rising edge
            RIGHTBounds[0] -= change;
        }
        if(gamepad1.dpad_up && previousDpadU == false){
            RIGHTBounds[0] += change;
        }
        if(gamepad1.dpad_down && previousDpadD == false){
            RIGHTBounds[1] -= change;
        }

        if(gamepad1.dpad_right && previousDpadR == false){
            RIGHTBounds[1] += change;
        }
        //0.7 -> 0.3 left
        //0.25 -> 0.65 right

        if(gamepad1.left_bumper){
            armServoLeft.setPosition(0.3);
            armServoRight.setPosition(0.65);
        }

        if(gamepad1.right_bumper){
            armServoLeft.setPosition(0.7);
            armServoRight.setPosition(0.25);
        }
        if(gamepad1.left_trigger > 0){
            grabServo.setPosition(grabOpen);
        }

        if(gamepad1.right_trigger > 0){
            grabServo.setPosition(grabClosed);
        }

        //ARM SERVO LEFT OPEN CLOSE
        if(gamepad1.left_stick_x < -0.5){
            armServoLeft.setPosition(LEFTBounds[0]);
        }
        if(gamepad1.left_stick_x > 0.5){
            armServoLeft.setPosition(LEFTBounds[1]);
        }
        //ARM SERVO RIGHT OPEN CLOSE
        if(gamepad1.right_stick_x < -0.5){
            armServoRight.setPosition(RIGHTBounds[0]);
        }
        if(gamepad1.right_stick_x > 0.5){
            armServoRight.setPosition(RIGHTBounds[1]);
        }


        previousDpadD = gamepad1.dpad_down;
        previousDpadL = gamepad1.dpad_left;
        previousDpadR = gamepad1.dpad_right;
        previousDpadU = gamepad1.dpad_up;
    }

    public void logServoPos(){
        telemetry.addData("Left Bounds Closed", LEFTBounds[0]);
        telemetry.addData("Left Bounds Open", LEFTBounds[1]);
        telemetry.addData("Right Bounds Closed", RIGHTBounds[0]);
        telemetry.addData("Right Bounds Open", RIGHTBounds[1]);



        telemetry.addData("Arm Servo Left Pos", armServoLeft.getPosition());
        telemetry.addData("Arm Servo Right Pos", armServoRight.getPosition());
//        telemetry.addData("Grab Servo Pos", grabServo.getPosition());

    }
}

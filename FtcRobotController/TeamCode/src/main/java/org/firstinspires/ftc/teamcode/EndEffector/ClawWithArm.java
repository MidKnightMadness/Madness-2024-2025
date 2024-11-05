package org.firstinspires.ftc.teamcode.EndEffector;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;


@Config
@TeleOp(name = "ClawWithArm")
public class ClawWithArm extends OpMode {
    Servo grabServo;
    Servo armServo;
    ColorSensor colorSensor;

    ColorSensorWrapper colorSensorWrapper;
    Timer timer;
    public static double grabOpen = 0.4;
    public static double grabClosed = 0.635;
    public static double grabNeutral = 0.4;

    //need fixing
    public static double armDown = 0.0;
    public static double armUp = 0.0;
    public static double armNeutral = 0.0;
    SampleColors.Colors detected = SampleColors.Colors.NONE;
    @Override
    public void init() {
        grabServo = hardwareMap.get(Servo.class, "grab servo");
        armServo = hardwareMap.get(Servo.class, "arm servo");

        timer = new Timer();
        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, 2);

        grabServo.setPosition(grabNeutral);
        armServo.setPosition(armNeutral);

    }

    @Override
    public void loop() {

        logServoPos();
        telemetry.addData("Current Time", timer.updateTime());
        telemetry.addData("Update Rate", 1 /timer.getDeltaTime());

        colorSensorWrapper.update();
        RGBColor rgbColor = colorSensorWrapper.getValue();

        detected = ColorClassifier.classify(rgbColor);
        telemetry.addData("Color Classification", detected);
        telemetry.addData("RGBA", rgbColor.toString());

        if(gamepad1.left_bumper){
            armServo.setPosition(armDown);
        }
        if(gamepad1.right_bumper){
            armServo.setPosition(armUp);
        }

        if(gamepad1.left_stick_x < -0.5){
            grabServo.setPosition(grabOpen);
        }
        if(gamepad1.right_stick_x > 0.5){
            grabServo.setPosition(grabClosed);
        }

        if(gamepad1.right_stick_x < -0.5){
            grabServo.setPosition(grabNeutral);
        }

        if (gamepad1.left_stick_x > 0.5) {
            armServo.setPosition(armNeutral);
        }


    }

    public void logServoPos(){
        telemetry.addData("Arm Servo Pos", armServo.getPosition());
        telemetry.addData("Grab Servo Pos", grabServo.getPosition());

    }
}

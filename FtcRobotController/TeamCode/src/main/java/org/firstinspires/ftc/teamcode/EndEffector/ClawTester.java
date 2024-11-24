package org.firstinspires.ftc.teamcode.EndEffector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;

//@TeleOp(name = "Claw")

public class ClawTester extends OpMode {
    ServoSmooth servoSmooth;
    Timer timer;
    Timer ServoTimer;




    double LEFT_BOUNDS = 0.4;
    double RIGHT_BOUNDS = 0.635;
    double NEUTRAL_VALUE = 0.4;
    double change = 0.015;
    double wristDown = 0; //change this val
    double wristNeutral = 0;//change this val
    //double MARGIN_OF_ERROR = 0.05; //arbitrary

//    FtcDashboard dashboard;
//    TelemetryPacket packet;
    

    ColorSensor colorSensor;
    ColorSensorWrapper colorSensorWrapper;
    public int bufferSize = 3;

    SampleColors.Colors detected = SampleColors.Colors.NONE;

    @Override
    public void init(){
        servoSmooth = new ServoSmooth(hardwareMap, telemetry);
        ServoTimer = new Timer();


        try {
            servoSmooth.setToPosition(NEUTRAL_VALUE);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Servo Position: ", servoSmooth.getPosition());
        position = servoSmooth.getPosition();

        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, bufferSize);


        
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


    double previousPosition = 0;

    public static double Kp = 0.1;

    public static double position;


    @Override
    public void loop() {


        colorSensorWrapper.update();
        RGBColor rgbColor = colorSensorWrapper.getValue();

        detected = ColorClassifier.classify(rgbColor);

        telemetry.addData("RGB Values", rgbColor.toString());
        telemetry.addData("Detected Color", detected);


        //servo.
        //boolean a .update(gamepad1.a);
//        packet.fieldOverlay().setFill("gray").fillRect(-15, 15, 20, 20);
//        telemetryPacket = ftcDashboard.getTelemetry();
//
//        telemetryPacket.update();

//        if (gamepad1.left_bumper) {//set to max closed bounds
//            ServoTimer = new Timer();
//            //targetPos += change;
////            try {
////                servoSmooth.setToPosition(LEFT_BOUNDS);
////            } catch (InterruptedException e) {
////                throw new RuntimeException(e);
////            }
////            servo.setPosition(servo.getPosition()+change);
//
//            double error = servoSmooth.getPosition() - LEFT_BOUNDS;
//            while(error > 0.01){
//
//                double value = servoSmooth.motionProfiledSetPosition(Math.abs(LEFT_BOUNDS - servoSmooth.getPosition()), 250, 500, ServoTimer.updateTime());
//
//                position = -1 * (servoSmooth.getPosition() - value) * Kp;
//
//                servoSmooth.servo.setPosition(position);
//            }
//
//        }

        if (gamepad1.right_bumper && previousGamepadRB == false) {//set to open bounds
            //targetPos -= change;
//            try {
//                servoSmooth.setToPosition(RIGHT_BOUNDS);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }

            ServoTimer = new Timer();
            closeRight = true;
        }

        if(closeRight == true){
            double error = RIGHT_BOUNDS - servoSmooth.getPosition();

            while(error > 0.01) {
                error = RIGHT_BOUNDS - servoSmooth.getPosition();
                telemetry.addData("Error", error);
                double value = servoSmooth.motionProfiledSetPosition(Math.abs(RIGHT_BOUNDS - NEUTRAL_VALUE), 1, 1, ServoTimer.updateTime(), telemetry);

                position = (value + LEFT_BOUNDS);
                double power = position;
                telemetry.addData("Time", ServoTimer.updateTime());
                telemetry.addData("Power", power);
                telemetry.update();
                servoSmooth.setDirectly(power);

                ServoTimer.updateTime();
                double deltaTime = ServoTimer.getDeltaTime();
//                packet.put("Delta Time", deltaTime);
//                packet.put("Servo Position", servoSmooth.getPosition() - previousPosition);
//                packet.put("Servo Velocity", (servoSmooth.getPosition() - previousPosition)/ deltaTime);
//                packet.put("Servo Acceleration", ((servoSmooth.getPosition() - previousPosition) / deltaTime) / deltaTime);
//                dashboard.sendTelemetryPacket(packet);
            }
                closeRight = false;

        }

        if(gamepad1.right_stick_x > 0.5){
            servoSmooth.setDirectly(RIGHT_BOUNDS);
        }

        if(gamepad1.left_stick_x > 0.5){
            servoSmooth.setDirectly(LEFT_BOUNDS);
        }


        previousGamepadRB = gamepad1.right_bumper;
        previousGamepadLB = gamepad1.left_bumper;

        if(gamepad1.dpad_left && previousDpadL == false ){ //rising edge
            LEFT_BOUNDS -= change;
        }
        if(gamepad1.dpad_up && previousDpadU == false){
            LEFT_BOUNDS += change;
        }
        if(gamepad1.dpad_down && previousDpadD == false){
            RIGHT_BOUNDS -= change;
        }

        if(gamepad1.dpad_right && previousDpadR == false){
            RIGHT_BOUNDS += change;
        }






        telemetry.addData("SetPosition", position);
            telemetry.addData("CloseLeft", closeLeft);
            telemetry.addData("CloseRight", closeRight);
            telemetry.addData("Gamepad Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Gamepad Right Bumper", gamepad1.right_bumper);

            telemetry.addData("Left Bound", LEFT_BOUNDS);
            telemetry.addData("Right Bound", RIGHT_BOUNDS);

        ServoTimer.updateTime();
        double deltaTime = ServoTimer.getDeltaTime();
//        packet.put("Delta Time", deltaTime);
//        packet.put("Servo Position", servoSmooth.getPosition() - previousPosition);
//        packet.put("Servo Velocity", (servoSmooth.getPosition() - previousPosition)/ deltaTime);
//        packet.put("Servo Acceleration", ((servoSmooth.getPosition() - previousPosition) / deltaTime) / deltaTime);
//        dashboard.sendTelemetryPacket(packet);
//        servo.setPosition(targetPos);
//


//        if(gamepad1.x){
//            servo.setPosition(LEFT_BOUNDS);
//        }
//
//        if(gamepad1.y){
//            servo.setPosition(RIGHT_BOUNDS);
//        }


//        if(gamepad1.a){
//            servo.setPosition(servo.getPosition() - change);
//        }
//        if(gamepad1.b){
//            servo.setPosition(servo.getPosition() + change);
//        }
//

        previousDpadD = gamepad1.dpad_down;
        previousDpadL = gamepad1.dpad_left;
        previousDpadR = gamepad1.dpad_right;
        previousDpadU = gamepad1.dpad_up;

        previousPosition = servoSmooth.getPosition();

        telemetry.addLine("Servo Position" + servoSmooth.getPosition());
        telemetry.addData("Change: ", change);
       // telemetry.addLine("Time: " + ServoTimer.updateTime());

        telemetry.update();

    }
}

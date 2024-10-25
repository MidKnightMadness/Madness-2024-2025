package org.firstinspires.ftc.teamcode.EndEffector;

import static android.provider.SyncStateContract.Helpers.update;

import android.widget.Button;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp(name = "Claw")
@Config
public class ClawTester extends OpMode {
    Servo servo;
    Timer timer;

    double LEFT_BOUNDS = 0.4;
    double RIGHT_BOUNDS = 0.635;
    double NEUTRAL_VALUE = 0.4;
    double change = 0.015;
    double wristDown = 0; //change this val
    double wristNeutral = 0;//change this val
    //double MARGIN_OF_ERROR = 0.05; //arbitrary

    FtcDashboard ftcDashboard;
    TelemetryPacket packet;
    Telemetry telemetryPacket;
    double position;
    ColorSensor colorSensor;
    ColorSensorWrapper colorSensorWrapper;
    public int bufferSize = 3;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "claw grabber");
        timer = new Timer();

        servo.setPosition(NEUTRAL_VALUE);
        telemetry.addData("Servo Position: ", servo.getPosition());
        position = servo.getPosition();

        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, bufferSize);


    }



    boolean previousDpadL;
    boolean previousDpadR;
    boolean previousDpadU;
    boolean previousDpadD;
    //double targetPos = 0.5;

    @Override
    public void loop() {

        colorSensorWrapper.update();
        RGBColor rgbColor = colorSensorWrapper.getValue();

        telemetry.addData("RGB Values", rgbColor.toString());
        telemetry.addData("Time", timer.updateTime());


        //servo.
        //boolean a .update(gamepad1.a);
//        ftcDashboard = FtcDashboard.getInstance();
//        packet.fieldOverlay().setFill("gray").fillRect(-15, 15, 20, 20);
//        telemetryPacket = ftcDashboard.getTelemetry();
//
//        telemetryPacket.update();

        if (gamepad1.left_bumper) {//set to max closed bounds
            //targetPos += change;
            servo.setPosition(LEFT_BOUNDS);
//            servo.setPosition(servo.getPosition()+change);

        }


        if (gamepad1.right_bumper) {//set to open bounds
            //targetPos -= change;
                servo.setPosition(RIGHT_BOUNDS);
        }
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

            telemetry.addData("Gamepad Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Gamepad Right Bumper", gamepad1.right_bumper);

            telemetry.addData("Left Bound", LEFT_BOUNDS);
            telemetry.addData("Right Bound", RIGHT_BOUNDS);
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

            telemetry.addLine("Servo Position" + servo.getPosition());
            telemetry.addData("Change: ", change);
            telemetry.addLine("Time: " + timer.updateTime());


            telemetry.update();
        }
}

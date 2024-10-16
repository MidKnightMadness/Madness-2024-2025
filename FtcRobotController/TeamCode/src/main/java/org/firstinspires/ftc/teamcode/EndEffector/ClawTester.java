package org.firstinspires.ftc.teamcode.EndEffector;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Timer;

@TeleOp(name = "Claw")
@Config
public class ClawTester extends OpMode {
    Servo servo;
    Timer timer;

    double LEFT_BOUNDS = 0.1;
    double RIGHT_BOUNDS = 0.1;
    double NEUTRAL_VALUE = 0.4;
    double change = 0.01;
    double wristDown = 0; //change this val
    double wristNeutral = 0;//change this val
    //double MARGIN_OF_ERROR = 0.05; //arbitrary

    FtcDashboard ftcDashboard;
    TelemetryPacket packet;
    Telemetry telemetryPacket;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "claw grabber");
        timer = new Timer();

        servo.setPosition(NEUTRAL_VALUE);
        telemetry.addData("Servo Position: ", servo.getPosition());


    }




    @Override
    public void loop() {

        ftcDashboard = FtcDashboard.getInstance();
        packet.fieldOverlay().setFill("gray").fillRect(-15, 15, 20, 20);
        telemetryPacket = ftcDashboard.getTelemetry();

        telemetryPacket.update();

        if(gamepad1.a){//set to max closed bounds
            while(servo.getPosition() > LEFT_BOUNDS){
                servo.setPosition(servo.getPosition()- change);
            }
        }

        else if(gamepad1.b){//set to open bounds
            while(servo.getPosition() < RIGHT_BOUNDS){
                servo.setPosition(servo.getPosition() + change);
            }
        }


        if(gamepad1.x){
            servo.setPosition(LEFT_BOUNDS);
        }

        if(gamepad1.y){
            servo.setPosition(RIGHT_BOUNDS);
        }



//        if(gamepad1.a){
//            servo.setPosition(servo.getPosition() - change);
//        }
//        if(gamepad1.b){
//            servo.setPosition(servo.getPosition() + change);
//        }
//


        telemetry.addLine("Servo Position" +  servo.getPosition());
        telemetry.addData("Change: ", change);
        telemetry.addLine("Time: " + timer.updateTime());



        telemetry.update();
    }


}

package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.Timer;

public class ClawTester extends OpMode {
    Servo servo;
    Timer timer;

    double LEFT_BOUNDS = 0.3;
    double RIGHT_BOUNDS = 0.7;
    double NEUTRAL_VALUE = 0.56;
    double change = 0.1;
    double wristDown = 0; //change this val
    double wristNeutral = 0;//change this val
    double MARGIN_OF_ERROR = 0.05; //arbitrary

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "clawGrabber");
        timer = new Timer();
    }



    @Override
    public void loop() {
        //rework this depending on how the claw is made; if there is a single servo for claw remove right_bounds
        //also add code for wrist
        /*
        if(gamepad1.y && servo.getPosition() == wristDown ){// +/- margin of error*
             servo.setPosition(wristNeutral);
         }
        else if (gamepad1.y && servo.getPosition() == wristDown ){// +/- margin of error*
              servo.setPosition(downBound);
        }*/
        
        if(gamepad1.dpad_left){
            servo.setPosition(servo.getPosition() + change);
        }

        if(gamepad1.dpad_right){
            servo.setPosition(servo.getPosition() - change);
        }

        
        if(gamepad1.x){
            servo.setPosition(LEFT_BOUNDS);
            
        }

        if(gamepad1.y){
            servo.setPosition(RIGHT_BOUNDS);
        }

        if(gamepad1.a){
            servo.setPosition(NEUTRAL_VALUE);
        }

        telemetry.addLine("Servo Position: " + servo.getPosition());
        telemetry.addLine("Change: " + change);
        telemetry.addLine("Time: " + timer.updateTime());
    }


}

package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.ButtonToggle;
import org.firstinspires.ftc.teamcode.Helper.Timer;


public class HorizontalSlides {
    Servo horizontalRight;
    Servo horizontalLeft;
    Timer timer;

    public double[] servoLeftBounds = new double[]{5, 0};
    public double[] servoRightBounds = new double[]{0, 5};


    double leftSpeed = 0.5;
    double rightSpeed = 0.7;

    public HorizontalSlides(HardwareMap hardwareMap){
        horizontalLeft = hardwareMap.get(Servo.class, "Horizontal Slides Left");
        //horizontalRight = hardwareMap.get(Servo.class, "Horizontal Slides Right");

        timer = new Timer();
    }

    public void extend(){
        horizontalLeft.setPosition(servoLeftBounds[1]);
        //horizontalRight.setPosition(percent * servoRightBounds[1]);
//        horizontalLeft.setPosition(horizontalLeft.getPosition() + 0.1*x);
//        horizontalRight.setPosition(horizontalRight.getPosition() + 0.1*x);
    }
    public void close(){
        horizontalLeft.setPosition(servoLeftBounds[0]);
      //  horizontalLeft.setPosition(servoRightBounds[0]);
    }

    public double getLeftPosition(){
        return horizontalLeft.getPosition();
    }




}

package org.firstinspires.ftc.teamcode.Slides;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.Timer;



public class HorizontalSlides {
    Servo horizontalRight;
    Servo horizontalLeft;
    Timer timer;

    public double[] servoLeftBounds = new double[]{0, 202};
    public double[] servoRightBounds = new double[]{0, 202};


    double leftSpeed = 0.5;
    double rightSpeed = 0.7;

    public HorizontalSlides(HardwareMap hardwareMap){
        horizontalRight = hardwareMap.get(Servo .class, "Horizontal Slides Right");
        horizontalLeft = hardwareMap.get(Servo.class, "Horizontal Slides Left");

        timer = new Timer();
    }

    public void extend(double x){
//        horizontalLeft.setPosition(servoLeftBounds[1]);
//        horizontalRight.setPosition(servoRightBounds[1]);
        horizontalLeft.setPosition(horizontalLeft.getPosition() + 0.1*x);
        horizontalRight.setPosition(horzontalRight.getPosition() + 0.1*x);
    }





}

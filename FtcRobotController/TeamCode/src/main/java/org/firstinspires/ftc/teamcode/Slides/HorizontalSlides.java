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
    double maxEntensionTime = 2.5;
    boolean rumble = false;

    HardwareMap hardwareMap;
    public double[] leftServoBounds = new double[]{0, 0.7};

    public HorizontalSlides(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        horizontalLeft = hardwareMap.get(Servo.class, "Horizontal Slides Left");

        //horizontalRight = hardwareMap.get(Servo.class, "Horizontal Slides Right");

    }

    public void extend(double percent) {
        timer = new Timer();
        double totalTime = percent * maxEntensionTime;
        double currentTime = timer.updateTime();

        while (currentTime < totalTime) {
            currentTime = timer.updateTime();
            horizontalLeft.setDirection(Servo.Direction.FORWARD);
            horizontalLeft.setPosition(Servo.MAX_POSITION);
            if(totalTime - currentTime < 0.5){
                rumble = true;
            }
        }
        rumble = false;
        horizontalLeft.setPosition(0.5);

//        horizontalRight.setPosition(percent * servoRightBounds[1]);
//        horizontalLeft.setPosition(horizontalLeft.getPosition() + 0.1*x);
//        horizontalRight.setPosition(horizontalRight.getPosition() + 0.1*x);
    }

    public void retract(double percent){
        timer = new Timer();
        double totalTime = percent * maxEntensionTime;
        double currentTime = timer.updateTime();

        while (currentTime < totalTime) {
            currentTime = timer.updateTime();
            horizontalLeft.setDirection(Servo.Direction.FORWARD);
            horizontalLeft.setPosition(Servo.MIN_POSITION);
            if(totalTime - currentTime < 0.5){
                rumble = true;
            }
        }
        rumble = false;
        horizontalLeft.setPosition(0.5);
      //  horizontalLeft.setPosition(servoRightBounds[0]);
    }

    boolean isRumble(){
        return rumble;
    }


    public void stop(){
        horizontalLeft.setPosition(0.5);

    }
    public double getLeftPosition(){
        return horizontalLeft.getPosition();
    }






}

package org.firstinspires.ftc.teamcode.EndEffector;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Helper.Timer;

public class ServoSmooth {
    Servo servo;
    Timer timer;
    Telemetry telemetry;

    double change = 0.01;
    long waitTime = 2;
    //change
    double maxVelocity = 0;
    double maxAcceleration = 0;



    public ServoSmooth(HardwareMap hardwareMap, Telemetry telemetry){
        servo = hardwareMap.get(Servo.class, "claw grabber");
        timer = new Timer();
        this.telemetry = telemetry;
    }

    public void setDirectly(double targetPosition){
        servo.setPosition(targetPosition);
    }

    //direct run to position
    public void setToPosition(double targetPosition) throws InterruptedException {
        if(servo.getPosition() +0.01 < targetPosition) {
            while (servo.getPosition() + 0.01 < targetPosition) {
                servo.setPosition(servo.getPosition() + change);
                Thread.sleep(waitTime);
            }
        }
        else if(servo.getPosition() -0.01 > targetPosition){
            while(servo.getPosition() - 0.01 > targetPosition){
                servo.setPosition(servo.getPosition() - change);
                Thread.sleep(waitTime);
            }
        }

    }

    //returns the value applied to servo to apply trapezoidial curve
    public double motionProfiledSetPosition(double deltaPosition, double maxVelocity, double maxAcceleration, double currentTime, Telemetry telemetry){
        double accelerationTime = maxVelocity / maxAcceleration;
        double halfwayDist = deltaPosition / 2;


        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        //if it accelerates past halfway point, accelerate as much as possible in that halfway dist

        if(accelerationDistance > halfwayDist){
            telemetry.addLine("True");
            //recalculate time to accelerate
            accelerationTime = Math.sqrt(halfwayDist * 2 / maxAcceleration);

            //recalculate acceleration distance,
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            //recalculate max Velocity
            maxVelocity = maxAcceleration * accelerationTime;
        }

        //decceleration variables
        double decelerationTime = accelerationTime;
        double decelerationDistance = accelerationDistance;

        //flat horizontal line of trapezoidial curve
        double flatDistance = Math.abs(deltaPosition - accelerationDistance - decelerationDistance);
        telemetry.addData("MaxVelocity", maxVelocity);
        telemetry.addData("AccelerationDist" ,accelerationDistance);
        telemetry.addData("DecelerationDist", decelerationDistance);
        telemetry.addData("FlatDist", flatDistance);
        double flatTime = flatDistance / maxVelocity;


        double entireTime = accelerationTime + flatTime + decelerationTime;

        telemetry.addLine("Target Point" + deltaPosition);
        telemetry.addLine("Entire Time" + entireTime);
        telemetry.addLine("Accelerating" + accelerationTime);
        telemetry.addLine("Flat" + flatTime);
        telemetry.addLine("Decelerating" + decelerationTime);
        if(currentTime < accelerationTime){
            //how much distance covered during acceleration phase
            return 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
        }

        else if(currentTime < (accelerationTime + flatTime)){
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            return (accelerationDistance + maxVelocity * (currentTime - accelerationTime));
        }

        else if(currentTime >= (accelerationTime + flatTime) && currentTime < entireTime){
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
            flatDistance = maxVelocity * flatTime;
            //deceleration rather than acceleration so - 1/2 instead of +1/2
            decelerationDistance = maxVelocity * (currentTime - (flatTime + accelerationTime)) - (0.5 * maxAcceleration * Math.pow(currentTime - (flatTime + accelerationTime), 2));
            return (accelerationDistance + flatDistance + decelerationDistance);
        }

        else{
            telemetry.addLine("Complete");
            return deltaPosition;
        }

    }

    public double getPosition(){
        return servo.getPosition();
    }
    public void setWaitTime(long waitTime){
        this.waitTime = waitTime;
    }
    public void getTelemetry(){
        telemetry.addData("Servo Pos", servo.getPosition());
        telemetry.addData("Wait Time", waitTime);
        telemetry.addData("Change", change);
    }




}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Far Auto")
public class FarAuto extends AutoDrive{

    @Override
    public void park() {

        try {
            Thread.sleep(8000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        deadReckoningDrive.setMotorVelocitiesForTime(timeForward, 0, 0.5, 0, 1000, telemetry);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        deadReckoningDrive.setMotorVelocitiesForTime(5, 0.5, 0, 0, 1000, telemetry);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        deadReckoningDrive.setMotorVelocitiesForTime(timeBack, 0, -0.5, 0, 1000, telemetry);
    }

    @Override
    public void loop(){

//        if(timer.updateTime() < timeForward) {
//            mecanumDrive.velocityDrive(0, 0.4, 0, 1000);
//        }
//
//        try {
//            Thread.sleep(3000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        //backward
//
//        if(timer.updateTime() > (timeForward ) && timer.updateTime() < (timeForward + timeBack )) {
//            mecanumDrive.velocityDrive(0, -0.3, 0, 1000);
//        }
//
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        if(timer.updateTime() > (timeForward + timeBack ) && timer.updateTime() < (timeForward + timeBack + timeRight)){
//            mecanumDrive.velocityDrive(0.3, 0, 0, 1000);
//        }
//        else {
//            mecanumDrive.setPowers(0, 0, 0, 0, 0);
//        }
//
//        //wait
//        try {
//            Thread.sleep(2500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        if(timer.updateTime() > (timeRight + timeForward + timeBack )) {
//            mecanumDrive.setPowers(0, 0, 0, 0, 0);
//        }
    }

}

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Push Far Auto")
public class FarAutoPush extends AutoDrive{


    double sleepTime = 5000;

    @Override
    public void park() {
        try {
            Thread.sleep((long) sleepTime);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        deadReckoningDrive.setMotorVelocitiesForTime(13, 0.8, 0, 0, 1000, telemetry);
    }
    @Override
    public void loop(){

    }

}

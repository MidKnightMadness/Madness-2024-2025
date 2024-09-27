package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {

    ElapsedTime elapsedTime;
    double previousTime;
    double currentTime;
    double deltaTime;

    public Timer(){
        elapsedTime = new ElapsedTime();
        elapsedTime.startTime();
    }

    public double getDeltaTime(){
        return deltaTime;
    }

    public double getPreviousTime(){
        return previousTime;
    }



    public double updateTime(){
        currentTime = elapsedTime.time();
        deltaTime = currentTime - previousTime;
        previousTime = currentTime;

        return currentTime;
    }

}

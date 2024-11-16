package org.firstinspires.ftc.teamcode.DistanceSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class DistanceSensorBuffer {
    double BUFFER_SIZE = 3;
    ArrayList<Double> distanceBuffer = new ArrayList<>();
    DistanceSensor distanceSensor;
    double averageDistance;
    public DistanceSensorBuffer(DistanceSensor distanceSensor, double bufferSize){
        BUFFER_SIZE = bufferSize;
        this.distanceSensor = distanceSensor;
    }

    public double getBufferSize(){
        return BUFFER_SIZE;
    }

    public void setBufferSize(double bufferSize){
        BUFFER_SIZE = bufferSize;
    }

    public void update(){
        distanceBuffer.add(distanceSensor.getDistance(DistanceUnit.INCH));
        if(distanceBuffer.size() == BUFFER_SIZE){
            averageDistance = 0;
            for(int i = 0; i < BUFFER_SIZE; i++){
                averageDistance += distanceBuffer.get(i);
            }
            averageDistance = (double) averageDistance / BUFFER_SIZE;
            distanceBuffer.clear();
        }
    }

    public double getNormalizedDistance(){
        return averageDistance;
    }
}


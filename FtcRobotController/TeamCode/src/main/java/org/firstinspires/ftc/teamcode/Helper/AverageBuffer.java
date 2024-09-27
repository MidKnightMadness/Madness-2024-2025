package org.firstinspires.ftc.teamcode.Helper;

import java.util.ArrayList;
import java.util.List;

public class AverageBuffer {
    public ArrayList<Double> list;

    private double BUFFER_SIZE = 2;
    public double average;

    public AverageBuffer(int size){
        this.BUFFER_SIZE = size;
        list = new ArrayList<>();

    }

    public void addValue(double val){
        list.add(val);
        if(list.size() > BUFFER_SIZE){
            this.average = getAverage();
        }
    }

    private double getAverage() {
        int sum = 0;
        for(int i = 0; i < list.size(); i++){
            sum += list.get(i);
        }
        sum /= BUFFER_SIZE;
        return sum;
    }
    public void reset(){
        list.clear();
    }

    public double getVal(){
        return average;
    }

}

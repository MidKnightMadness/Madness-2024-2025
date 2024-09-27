package org.firstinspires.ftc.teamcode.Helper;

public class Coordinate {//excluding angle
    double x = 0;
    double y = 0;


    public Coordinate(double x, double y){
        this.x = x;
        this.y = y;
    }

    public void updateCoordinate(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

}

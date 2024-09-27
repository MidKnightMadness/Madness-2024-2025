package org.firstinspires.ftc.teamcode.Helper;

public class Vector {
    double x;
    double y;
    double magnitude;

    public Vector(int x, int y){
        this.x = x;
        this.y = y;
        this.magnitude = Math.sqrt(x^2 + y^2);
    }

    public double getMagnitude(){
        return magnitude;
    }

    public double xCoordinate(){
        return x;
    }

    public double yCoordinate(){
        return y;
    }



}

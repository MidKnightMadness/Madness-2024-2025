package org.firstinspires.ftc.teamcode.Drive;


public class Coordinate {

    double xCord, yCord, angle;

    public void setXCoordinate(double newX){
        xCord = newX;
    }
    public void setYCoordinate(double newY){
        yCord = newY;
    }
    public void setXYCoordinats(double newX, double newY){
        xCord = newX;
        yCord = newY;
    }
    public void setAngle(double newAngle) {
        angle = newAngle;
    }
    public double getXCoordinate(){
        return xCord;
    }
    public double getYCoordinate(){
        return yCord;
    }

    public double getAngle(){
        return angle;
    }


}
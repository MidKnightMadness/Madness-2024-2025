package org.firstinspires.ftc.teamcode.Helper;

public class Pose {
    double x;
    double y;
    double rotationRadians;


    public Pose(double x, double y, double rotationRadians){
        this.x = x;
        this.y = y;
        this.rotationRadians = rotationRadians;
    }


    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getRotationRadians(){
        return normalizeAngleRadians(rotationRadians);
    }
    public double getRotationDegrees(){
        return normalizeAngleDegrees(rotationRadians * 180 / Math.PI);
    }


    public double normalizeAngleRadians(double angle){
        return angle % (2 * Math.PI);
    }

    public double normalizeAngleDegrees(double angle){
        return angle % 360;
    }

}




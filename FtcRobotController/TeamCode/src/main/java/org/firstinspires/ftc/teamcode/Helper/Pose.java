package org.firstinspires.ftc.teamcode.Helper;

public class Pose {
    double x;
    double y;
    double rotation;


    public Pose(double x, double y, double rotation){
        this.x = x;
        this.y = y;
        this.rotation = rotation;
    }


    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getRotation(){
        return rotation;
    }

}




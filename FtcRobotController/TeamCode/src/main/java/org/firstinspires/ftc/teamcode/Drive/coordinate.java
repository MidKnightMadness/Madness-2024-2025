package org.firstinspires.ftc.teamcode.Drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class coordinate {

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
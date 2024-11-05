package org.firstinspires.ftc.teamcode.Pathing;


import org.opencv.core.Point;

import java.util.ArrayList;
import java.lang.Math.*;

import static java.lang.Math.*;

public class MathFunctions {

    public static double AngleWrap(double angle){
        while(angle < - Math.PI){
            angle = angle + 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle = angle - 2 * Math.PI;
        }

        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point CircleCenter, double radius, Point linePoint1, Point linePoint2){
        if(Math.abs(linePoint1.y - linePoint2.y) < 0.003){//doesn't matter difference too small
            linePoint1.y = linePoint2.y + 0.003;
        }

        if(Math.abs(linePoint1.x - linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint1.x);

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = linePoint1.x - CircleCenter.x;
        double y1 = linePoint1.y - CircleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1, 2) * x1);
        double quadraticC = ((pow(m1, 2) * pow(x1, 2))) - (2.0 * y1 * m1 * x1) + pow(y1, 2) - pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try {
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            //put back offset
            xRoot1 += CircleCenter.x;
            yRoot1 += CircleCenter.y;

            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;
            //don't need y because only need x to determine whether we're on it or not

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1, yRoot1));
            }


            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4.0  * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += CircleCenter.x;
            yRoot2 += CircleCenter.y;

            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2, yRoot2));
            }

        }
        catch(Exception e){//if line doesn't have an intersection


        }

        return allPoints;

    }



}

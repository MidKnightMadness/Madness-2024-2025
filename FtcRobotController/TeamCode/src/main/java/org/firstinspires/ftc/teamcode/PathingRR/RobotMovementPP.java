//package org.firstinspires.ftc.teamcode.Pathing;
//
//
//
//
//
//import java.util.ArrayList;
//
//import static com.company.Robot.*;
//import static RobotUtilities.MovementVars.*;
//import static teamcode.MathFunctions.AngleWrap;
//import static teamcode.MathFunctions.lineCircleIntersection;
//
//public class RobotMovementPP {
//    double xConstant = 10;
//    double yConstant = 10;
//    static ArrayList<CurvePoint> allPoint = new ArrayList<>();
//    static boolean endOfPath = false;
//
//    static boolean[] pointEdge = new boolean[allPoint.size()];
//    static int currentPointTracking = -1;
//
//
//    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle, boolean stop) throws InterruptedException{
//
//        allPoint = allPoints;
//        for (int i = 0; i < allPoints.size() - 1; i++) {
//            ComputerDebugging.sendLine(new FloatPoint(allPoints.get(i).x, allPoints.get(i).y),
//                    new FloatPoint(allPoints.get(i + 1).x, allPoints.get(i + 1).y));
//            //send information from program to debugger to display visually
//        }
//
//
//        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);
//
//
//        double distanceToTarget = Math.hypot(worldXPosition - allPoints.get(allPoints.size() - 1).x, worldYPosition - allPoints.get(allPoints.size() - 1).y);
//        //if follow point and final point are very close(near end of path)
//
////        if (distanceToTarget < 3) {//if near end of path
////            endOfPath = true;
////            CurvePoint lastPoint = allPoint.get(allPoints.size() - 1);
////            ComputerDebugging.sendKeyPoint(new FloatPoint(lastPoint.x, lastPoint.y));
////            goToPosition(lastPoint.x, lastPoint.y, lastPoint.moveSpeed, followAngle, lastPoint.turnSpeed);
////        } else {
//        ComputerDebugging.sendKeyPoint(new FloatPoint(followMe.x, followMe.y));
//        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed, stop);
//
//
//    }
//
//
//
//    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoint, Point robotLocation, double followRadius) {
//        CurvePoint followMe = new CurvePoint(pathPoint.get(0));
//        //default is to go to very first point in path if doesn't have intersections
//
//        for (int i = 0; i < pathPoint.size() - 1; i++) {//each time looking at a line, defined by two points, stop one early
//            CurvePoint startLine = pathPoint.get(i);
//            CurvePoint endLine = pathPoint.get(i + 1);
//
//            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(), endLine.toPoint());
//            //preferring points closer in order to endpoints
//
//            double closestAngle = 10000000;
//
//            for (Point thisIntersection : intersections) {
//                //absolute angle
//                double angle = Math.atan2(thisIntersection.y - worldYPosition, thisIntersection.x - worldXPosition);
//                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle - worldAngle_rad));
//
//                if (deltaAngle < closestAngle) {
//                    closestAngle = deltaAngle;
//                    followMe.setPoint(thisIntersection);
//
//                }
//            }
//        }
//
//        return followMe;
//    }
//
//    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed, boolean stop) throws InterruptedException {
//        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
//        System.out.println(worldYPosition);
//        double absoluteAngleToTarget = Math.atan2(y - worldYPosition, x - worldXPosition);
//
//        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));
//        //+ 90 degrees to make the robot angle relative back to x axis and not y axis before subtracting from absolute angle
//
//        //from the position of the robotpt
//        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
//        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;
//
//        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//        //normalize ratios of component power to total power
//
//        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
//
//        movement_x = movementXPower * movementSpeed;
//        movement_y = movementYPower * movementSpeed;
//
//        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
//        //subtract 180 starts from - y axis to + y axis
//        //as robot moves closer to target, it adjusts angle its best to match the preferred angle, more influence when closer
//
//        movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
//        //if the robot is off by more than 30 degrees, apply full turning power, clipped to nearest boundary
//
//        System.out.print(endOfPath);
//        //if close to target, stop turning -> fixes issues when really close to point and keeps turning
//        if(endOfPath){//10 cm
//            movement_turn = 0;
//            movement_x = 0;
//            movement_y = 0;
//        }
//        else if(stop == true){
//            movement_turn = 0;
//            movement_x = 0;
//            movement_y = 0;
//        }
//
//    }
//}
//

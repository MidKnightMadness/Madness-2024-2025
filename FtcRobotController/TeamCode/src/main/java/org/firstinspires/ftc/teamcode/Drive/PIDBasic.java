package org.firstinspires.ftc.teamcode.Drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class PIDBasic {
    ThreeWheelOdometry odometry;
    private Telemetry telemetry;
    //Madness Chassis Into the Deep
    double[] P = new double[]{0.01, 0.01, 0.01};
    double[] I = new double[]{0, 0, 0};
    double[] D = new double[]{0, 0, 0};
    double[] Kv = new double[]{0, 0, 0};
    double[] Ka = new double[]{0, 0, 0};

    //Madness Chassis 2 Years Ago[Test Robot]
    double[] targetStates = new double[3];
    double distancetoTarget;
    double initialDistanceToTarget;

    double[] PGains = new double[]{0,0,0};
    double[] IGains = new double[]{0,0,0};
    double[] DGains = new double[]{0,0,0};
    double[] FGains = new double[]{0,0,0};

    double maxIntegralSum = 0.5;


    public PIDBasic(ThreeWheelOdometry threeWheelOdometry, double targetX, double targetY, double targetAngle, Telemetry telemetry){
        this.odometry = threeWheelOdometry;

        initialDistanceToTarget = Math.sqrt(Math.pow(targetX - threeWheelOdometry.getXCoordinate(),2) + Math.pow(targetY- threeWheelOdometry.getYCoordinate(), 2));
        targetStates[0] = targetX;
        targetStates[1] = targetY;
        targetStates[2] = getRotationRadians(targetAngle);


        this.telemetry = telemetry;
    }

    public double getRotationDegrees(double angle){
        return normalizeAngle(angle * 180 / Math.PI);
    }
    public double getRotationRadians(double angle){
        return normalizeAngle(angle * Math.PI / 180);
    }


    double errorTotal[] = new double[]{0,0,0};


    double[] previousErrorValues = new double[3];
    double[] currentErrorValues = new double[3];
    double[] totalGains = new double[3];
    double integralBuildupConstantLeneincy = 0.01;
    public double[] updatePID(){
        odometry.update();

        distancetoTarget = Math.sqrt(Math.pow(targetStates[0] - odometry.getXCoordinate(),2)
                + Math.pow(targetStates[1] - odometry.getYCoordinate(), 2));

        telemetry.addData("Update Rate", odometry.getUpdateRate());

        telemetry.addData("Left Enc", odometry.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Enc", odometry.rightEncoder.getCurrentPosition());
        telemetry.addData("Front Enc", odometry.frontEncoder.getCurrentPosition());

        telemetry.addData("Robot X", odometry.getXCoordinate());
        telemetry.addData("Robot Y", odometry.getYCoordinate());
        telemetry.addData("Robot Theta", odometry.getRotationDegrees());

        telemetry.addData("Target X", targetStates[0]);
        telemetry.addData("Target Y", targetStates[1]);
        telemetry.addData("Target Theta", targetStates[2]);

        telemetry.addData("Initial Distance To Target", initialDistanceToTarget);
        telemetry.addData("Distance To Target", distancetoTarget);

        telemetry.addLine("----Gains-------");
        telemetry.addData("PX", PGains[0]);
        telemetry.addData("PY", PGains[1]);
        telemetry.addData("PTheta", PGains[2]);

        telemetry.addData("IX", IGains[0]);
        telemetry.addData("IY", IGains[1]);
        telemetry.addData("ITheta", IGains[2]);

        telemetry.addData("DX", DGains[0]);
        telemetry.addData("DY", DGains[1]);
        telemetry.addData("DTheta", DGains[2]);

        telemetry.addData("Total GainsX", totalGains[0]);
        telemetry.addData("Total GainsY", totalGains[1]);
        telemetry.addData("Total GainsTheta", totalGains[2]);

        //Calculate P Gains
        PGains[0] = (targetStates[0] - odometry.getXCoordinate()) * P[0];
        PGains[1] = (targetStates[1] - odometry.getYCoordinate()) * P[1];
        PGains[2] = - P[2] * normalizeAngle(targetStates[2] - odometry.getRotationRadians());

        errorTotal[0] += ((targetStates[0] - odometry.getXCoordinate()) * odometry.getUpdateRate());
        errorTotal[1] += ((targetStates[1] - odometry.getYCoordinate()) * odometry.getUpdateRate());
        errorTotal[2] += (normalizeAngle(targetStates[2] - odometry.getRotationRadians()) * odometry.getUpdateRate());

        currentErrorValues[0] = (targetStates[0] - odometry.getXCoordinate());
        currentErrorValues[1] = (targetStates[1] - odometry.getYCoordinate());
        currentErrorValues[2] = (targetStates[2] - odometry.getRotationRadians());//in degrees

        //Calculate I Gains
        IGains[0] = errorTotal[0] * I[0];
        IGains[1] = errorTotal[1] * I[1];
        IGains[2] = errorTotal[2] * I[2];

        //Integral Decay
        for(int i = 0; i < 3; i++){
            if(errorTotal[i] > maxIntegralSum){
                errorTotal[i] = maxIntegralSum;
            }
            else if(errorTotal[i] < -maxIntegralSum){
                errorTotal[i] = -maxIntegralSum;
            }

            //recalculate
            IGains[i] = errorTotal[i] * I[i];


            //if current pos is not equal to previous pos
            if(Math.abs(currentErrorValues[i] - previousErrorValues[i]) < integralBuildupConstantLeneincy){
                //Only applies when current error is very close to previous error value
                IGains[i] = 0;
            }
        }



        //Calculate D gains
        DGains[0] = D[0] * (odometry.getUpdateRate() * (currentErrorValues[0] - previousErrorValues[0]));
        DGains[1] = D[1] * (odometry.getUpdateRate() * (currentErrorValues[1] - previousErrorValues[1]));
        DGains[2] = D[2] * (odometry.getUpdateRate() * normalizeAngle(currentErrorValues[2] - previousErrorValues[2]));


        //Optional Feedforward value



        previousErrorValues[0] = currentErrorValues[0];
        previousErrorValues[1] = currentErrorValues[1];
        previousErrorValues[2] = currentErrorValues[2];


        totalGains[0] = PGains[0] + IGains[0] + DGains[0];
        totalGains[1] = PGains[1] + IGains[1] + DGains[1];
        totalGains[2] = PGains[2] + IGains[2] + DGains[2];

//        FGains[0] = Kv[0] * (referenceVelocity - odometryArc.getVelocityX()) * Ka[0] * (referenceAcceleration - odometryArc.getAccelerationX());
//        FGains[1] = Kv[1] * (referenceVelocity - odometryArc.getVelocityY()) * Ka[1] * (referenceAcceleration - odometryArc.getAccelerationY());

       // FGains[2] = Kv[1] * odometryArc. * Ka[1] * referenceAcceleration;


        //preserving the vectors

        double maxGains = 0.0;
        maxGains = Math.max(totalGains[0], Math.max(totalGains[1], totalGains[2]));

        if(Math.abs(maxGains) > 1){
            totalGains[0] /= maxGains;
            totalGains[1] /= maxGains;
            totalGains[2] /= maxGains;
        }

        return totalGains;
    }
    public double normalizeAngle(double angle){
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

}

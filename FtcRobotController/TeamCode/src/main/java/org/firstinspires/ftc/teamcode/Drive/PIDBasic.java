package org.firstinspires.ftc.teamcode.Drive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDBasic {
    OdometryArc odometryArc;
    private Telemetry telemetry;
    double[] P = new double[]{0, 0, 0};
    double[] I = new double[]{0, 0, 0};
    double[] D = new double[]{0, 0, 0};
    double[] Kv = new double[]{0, 0, 0};
    double[] Ka = new double[]{0, 0, 0};

    double[] targetStates = new double[3];
    double distancetoTarget;
    double initialDistanceToTarget;

    double[] PGains = new double[]{0,0,0};
    double[] IGains = new double[]{0,0,0};
    double[] DGains = new double[]{0,0,0};
    double[] FGains = new double[]{0,0,0};

    double maxIntegralSum = 0.5;


    public PIDBasic(OdometryArc odometryArc, double targetX, double targetY, double targetAngle, Telemetry telemetry){
        this.odometryArc = odometryArc;

        initialDistanceToTarget = Math.sqrt(Math.pow(targetX - odometryArc.getXCoordinate(),2) + Math.pow(targetY-odometryArc.getYCoordinate(), 2));
        targetStates[0] = targetX;
        targetStates[1] = targetY;
        targetStates[2] = targetAngle;


        this.telemetry = telemetry;
    }


    double errorTotal[] = new double[]{0,0,0};


    double[] previousErrorValues = new double[3];
    double[] currentErrorValues = new double[3];
    double[] totalGains = new double[3];
    double integralBuildupConstantLeneincy = 0.01;
    public double[] updatePID(){
        odometryArc.update();

        distancetoTarget = Math.sqrt(Math.pow(targetStates[0] - odometryArc.getXCoordinate(),2)
                + Math.pow(targetStates[1]-odometryArc.getYCoordinate(), 2));

        telemetry.addData("Update Rate", 1/odometryArc.getUpdateRate());

        telemetry.addData("Left Enc", odometryArc.leftEncoder.getCurrentPosition());
        telemetry.addData("Right Enc", odometryArc.rightEncoder.getCurrentPosition());
        telemetry.addData("Front Enc", odometryArc.frontEncoder.getCurrentPosition());

        telemetry.addData("Robot X", odometryArc.getXCoordinate());
        telemetry.addData("Robot Y", odometryArc.getYCoordinate());
        telemetry.addData("Robot Theta", odometryArc.getRotationDegrees());

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

        //Calculate P Gains
        PGains[0] = (targetStates[0] - odometryArc.getXCoordinate()) * P[0];
        PGains[1] = (targetStates[1] - odometryArc.getYCoordinate()) * P[1];
        PGains[2] = - P[2] * ((targetStates[2] - odometryArc.getRotationRadians()) % (2.0 * Math.PI));

        errorTotal[0] += ((targetStates[0] - odometryArc.getXCoordinate()) * odometryArc.getUpdateRate());
        errorTotal[1] += ((targetStates[1] - odometryArc.getYCoordinate()) * odometryArc.getUpdateRate());
        errorTotal[2] += ((targetStates[2] - odometryArc.getRotationRadians()) * odometryArc.getUpdateRate());

        currentErrorValues[0] = (targetStates[0] - odometryArc.getXCoordinate());
        currentErrorValues[1] = (targetStates[1] - odometryArc.getYCoordinate());

        //Integral Decay
        for(int i = 0; i < 3; i++){
            if(errorTotal[i] >= maxIntegralSum){
                errorTotal[i] = maxIntegralSum;
            }
            else if(errorTotal[i] <= maxIntegralSum){
                errorTotal[i] = -maxIntegralSum;
            }

            if(!(currentErrorValues[i] > (previousErrorValues[i] - integralBuildupConstantLeneincy) && currentErrorValues[i] < (previousErrorValues[i] + integralBuildupConstantLeneincy))){
                //Only applies when current error is very close to previous error value
                errorTotal[i] = 0;
            }
        }

        //Calculate I Gains
        IGains[0] = errorTotal[0] * I[0];
        IGains[1] = errorTotal[1] * I[1];
//        IGains[2] = errorTotal[2] * I[2];


        //Calculate D gains
        DGains[0] = D[0] * (odometryArc.getUpdateRate() * (targetStates[0] - odometryArc.getXCoordinate()));
        DGains[1] = D[1] * (odometryArc.getUpdateRate() * (targetStates[1] - odometryArc.getYCoordinate()));
//        DGains[2] = D[2] * (odometryArc.getUpdateRate() * (targetStates[2] - odometryArc.getRotationRadians()));


        //Optional Feedforward value



        previousErrorValues = currentErrorValues;


        totalGains[0] = PGains[0] + IGains[0] + DGains[0];
        totalGains[1] = PGains[1] + IGains[1] + DGains[1];
//        totalGains[2] = PGains[2] + IGains[2] + DGains[2];

//        FGains[0] = Kv[0] * (referenceVelocity - odometryArc.getVelocityX()) * Ka[0] * (referenceAcceleration - odometryArc.getAccelerationX());
//        FGains[1] = Kv[1] * (referenceVelocity - odometryArc.getVelocityY()) * Ka[1] * (referenceAcceleration - odometryArc.getAccelerationY());

       // FGains[2] = Kv[1] * odometryArc. * Ka[1] * referenceAcceleration;


        //preserving the vectors

        double maxGains = 0.0;
        maxGains = Math.max(totalGains[0], Math.max(totalGains[1], totalGains[2]));

        if(maxGains > 1){
            totalGains[0] /= maxGains;
            totalGains[1] /= maxGains;
            totalGains[2] /= maxGains;
        }

        return totalGains;
    }

}

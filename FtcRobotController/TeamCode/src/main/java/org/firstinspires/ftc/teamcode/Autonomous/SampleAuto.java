//package org.firstinspires.ftc.teamcode.Autonomous;
//
//public class SampleAuto extends AutoDrive {
//
//    double pTranslation = 1d / 12;
//    double pRotation = 1d / Math.PI;
//
//
//    double[] sampleX = new double[] {-12, -20, -28};
//
//    double samplePushY = 48;
//
//    @Override
//    public void park() {
//        errorDrive.driveToPosition(twoWheelOdometry, 0, 84, 0, 700, pTranslation , pRotation, 3);
//    }
//
//    @Override
//    public void start() {
//        errorDrive.driveToPosition(twoWheelOdometry, 0, samplePushY, 0, 700, pTranslation , pRotation, 3);
//        errorDrive.driveToPosition(twoWheelOdometry, sampleX[0], samplePushY, 0, 700, pTranslation , pRotation, 2);
//        for (int i = 0; i< sampleX.length; i++) {
//            errorDrive.driveToPosition(twoWheelOdometry, sampleX[i], 0, 0, 900, pTranslation , pRotation, 3);
//
//            if (i == 2) break;
//            errorDrive.driveToPosition(twoWheelOdometry, sampleX[i], samplePushY, 0, 700, pTranslation , pRotation, 4);
//        }
//
//        park();
//
//    }
//}

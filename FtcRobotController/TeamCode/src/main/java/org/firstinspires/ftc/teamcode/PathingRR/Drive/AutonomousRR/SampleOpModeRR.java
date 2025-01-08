//package org.firstinspires.ftc.teamcode.PathingRR.Drive.AutonomousRR;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
//import org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleMecanumDrive;
//
//@Config
//public class SampleOpModeRR extends LinearOpMode {
//
//    IMUWrapper imu;
//    double yaw;
//    CRServo leftArmServo;
//    CRServo rightArmServo;
//    Servo wristServo;
//    Servo clawServo;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        imu = new IMUWrapper(hardwareMap, telemetry);
//        imu.calibrateBiases();
//
//        leftArmServo = hardwareMap.get(CRServo.class, "leftArmServo");
//        rightArmServo = hardwareMap.get(CRServo.class, "rightArmServo");
//        wristServo = hardwareMap.get(Servo.class, "wristServo");
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//
//
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d startingPose = new Pose2d(0, 0, Math.PI / 2);
//        TrajectoryBuilder trajectoryBuilder = drive.trajectoryBuilder(startingPose)
//                .lineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(45)))
//                .addSpatialMarker(new Vector2d(-32,8), () -> {
//
//                }
//                )
//
//
//        waitForStart();
//
//        imu.update();
//        yaw = normalizeAngleRadians(imu.getYaw());
//    }
//
//    public double normalizeAngleRadians(double rotation){
//        return((rotation + 2 * Math.PI) % (2 * Math.PI));
//    }
//}

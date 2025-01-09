package org.firstinspires.ftc.teamcode.PathingRR.Drive.AutonomousRR;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PathingRR.TrajectorySequence.TrajectorySequence;

@Config
public class SampleBasketOpModeRR extends LinearOpMode {

    IMUWrapper imu;
    double yaw;
    CRServo leftArmServo;
    CRServo rightArmServo;
    Servo wristServo;
    Servo clawServo;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUWrapper(hardwareMap, telemetry);
        imu.calibrateBiases();

        //TODO: Recheck if there are more servos

        leftArmServo = hardwareMap.get(CRServo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(CRServo.class, "rightArmServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");


        // TODO: Set bounds of all servos


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startingPose = new Pose2d(0, 0, Math.PI / 2);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startingPose)
                //Drive to outake first sample into top basket
                .lineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(45)))
                .addSpatialMarker(new Vector2d(-32,8), () -> {
                }

                )


                .build();


        waitForStart();

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        drive.followTrajectorySequence(trajectorySequence);
        imu.update();
        yaw = normalizeAngleRadians(imu.getYaw());
    }

    public double normalizeAngleRadians(double rotation){
        return((rotation + 2 * Math.PI) % (2 * Math.PI));
    }
}

package org.firstinspires.ftc.teamcode.PathingRR.Drive.AutonomousRR;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Helper.IMUWrapper;
import org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.PathingRR.TrajectorySequence.TrajectorySequence;

@Config
public class SamplePushOpModeRR extends LinearOpMode {

    IMUWrapper imu;
    double yaw;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUWrapper(hardwareMap);



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startingPose = new Pose2d(0, 0, Math.PI / 2);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startingPose)
                //Drive to outake first sample into top basket
                .strafeTo(new Vector2d(-6, 60))
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

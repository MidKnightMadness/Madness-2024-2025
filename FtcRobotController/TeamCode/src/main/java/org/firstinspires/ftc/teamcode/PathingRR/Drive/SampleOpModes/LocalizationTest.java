package org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.PathingRR.Drive.SampleMecanumDrive;

@TeleOp(group = "drive", name = "LocalizationTest")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, telemetry);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.setPoseEstimate(new Pose2d(0.0, 0.0, Math.PI / 2));

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_x,
                            -gamepad1.left_stick_y,
                            gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", thetaToDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

    public double thetaToDegrees(double theta){
        return theta * 180 / Math.PI;
    }


}

package org.firstinspires.ftc.teamcode.EndEffector;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ColorSensor.ColorClassifier;
import org.firstinspires.ftc.teamcode.ColorSensor.ColorSensorWrapper;
import org.firstinspires.ftc.teamcode.ColorSensor.SampleColors;
import org.firstinspires.ftc.teamcode.Helper.RGBColor;
import org.firstinspires.ftc.teamcode.Helper.Timer;

//@TeleOp(name = "ClawWithArm2")
public class ClawWithArm extends OpMode {
    // Hardware components
    private Servo grabServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private ColorSensor colorSensor;

    // Helper classes
    private ColorSensorWrapper colorSensorWrapper;
    private Timer timer;

    // Servo positions
    public static double grabOpen = 0.4;
    public static double grabClosed = 0.8;
    public static double grabNeutral = 0.6; // Adjusted for neutral position
    private SampleColors.Colors detected = SampleColors.Colors.NONE;

    // Arm servo bounds
    private final double[] LEFTBounds = {0.3, 0.7}; // Closed, Open
    private final double[] RIGHTBounds = {0.65, 0.25}; // Closed, Open

    @Override
    public void init() {
        // Initialize hardware
        grabServo = hardwareMap.get(Servo.class, "claw grabber");
        armServoLeft = hardwareMap.get(Servo.class, "arm servo left");
        armServoRight = hardwareMap.get(Servo.class, "arm servo right");
        colorSensor = hardwareMap.get(ColorSensor.class, "claw color sensor");

        // Initialize helpers
        colorSensorWrapper = new ColorSensorWrapper(colorSensor, 2);
        timer = new Timer();

        // Set initial servo positions
        grabServo.setPosition(grabNeutral);
        armServoLeft.setPosition(LEFTBounds[0]);
        armServoRight.setPosition(RIGHTBounds[0]);
    }

    @Override
    public void loop() {
        // Update color sensor and telemetry
        colorSensorWrapper.update();
        RGBColor rgbColor = colorSensorWrapper.getValue();
        detected = ColorClassifier.classify(rgbColor);

        telemetry.addData("Color Classification", detected);
        telemetry.addData("RGBA", rgbColor.toString());
        telemetry.addData("Current Time", timer.updateTime());
        telemetry.addData("Update Rate", 1 / timer.getDeltaTime());

        // Control claw with triggers
        if (gamepad1.left_trigger > 0) {
            grabServo.setPosition(grabOpen); // Open the claw
        } else if (gamepad1.right_trigger > 0) {
            grabServo.setPosition(grabClosed); // Close the claw
        } else {
            grabServo.setPosition(grabNeutral); // Neutral position
        }

        // Control arm servos with D-Pad (both arms move together)
        if (gamepad1.dpad_up) {
            armServoLeft.setPosition(LEFTBounds[1]); // Open left arm
            armServoRight.setPosition(RIGHTBounds[1]); // Open right arm
        } else if (gamepad1.dpad_down) {
            armServoLeft.setPosition(LEFTBounds[0]); // Close left arm
            armServoRight.setPosition(RIGHTBounds[0]); // Close right arm
        }

        // Log servo positions
        logServoPos();
        telemetry.update();
    }

    private void logServoPos() {
        telemetry.addData("Arm Servo Left Pos", armServoLeft.getPosition());
        telemetry.addData("Arm Servo Right Pos", armServoRight.getPosition());
        telemetry.addData("Grab Servo Pos", grabServo.getPosition());
    }
}
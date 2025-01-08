package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

@TeleOp
public class Field_Centric_Mec extends LinearOpMode {

    SparkFunOTOS myOtos;
    double ropotaterPos;
    double speedMultiplier = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor front_left = hardwareMap.dcMotor.get("fl");
        DcMotor back_left = hardwareMap.dcMotor.get("rl");
        DcMotor front_right = hardwareMap.dcMotor.get("fr");
        DcMotor back_right = hardwareMap.dcMotor.get("rr");
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        CRServo roller = hardwareMap.crservo.get("roll");
        Servo ropeatater = hardwareMap.servo.get("rop");
        ropeatater.scaleRange(0.125, 0.875);
        ropeatater.setPosition(ropotaterPos);

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double armPower = 0;

            armPower = -gamepad2.right_stick_y;
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (gamepad2.right_trigger > 0.1) {
                roller.setPower(1);
            } else if (gamepad2.left_trigger > 0.1) {
                roller.setPower(-1);
            } else {
                roller.setPower(0);
            }

            if (gamepad2.right_bumper && ropotaterPos < 1) {
                ropotaterPos += 0.005;
            } else if (gamepad2.left_bumper && ropotaterPos > 0){
                ropotaterPos -= 0.005;
            }

            ropeatater.setPosition(ropotaterPos);

            front_left.setPower(frontLeftPower);
            back_left.setPower(backLeftPower);
            front_right.setPower(frontRightPower);
            back_right.setPower(backRightPower);
            arm.setPower(armPower * speedMultiplier);

            // Inform user of available controls
            telemetry.addLine("Press Y (triangle) on Gamepad to reset tracking");
            telemetry.addLine("Press X (square) on Gamepad to calibrate the IMU");
            telemetry.addLine();

            // Update the telemetry on the driver station
            telemetry.update();
        }
    }

}

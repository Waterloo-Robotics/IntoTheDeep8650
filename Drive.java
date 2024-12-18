package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Assigning this as a TeleOp OpMode called Drive*/
@TeleOp
@Disabled
public class Drive extends OpMode {

    /** Creating our 4 motors in the code. */
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;

    /* Setting a speed multiplier to adjust the speed of the robot. */
    double speedMultiplier = 0.67;

    /** Runs when the coach presses the INIT button. */
    public void init() {

        /** Assigning our motors to values in our hardware map on the driver station. */
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        rearLeft = hardwareMap.dcMotor.get("rl");
        rearRight = hardwareMap.dcMotor.get("rr");

        /** Setting all our motors to apply power to stop in place.*/
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /** Set directions of motors so forward is positive and backwards is negative when setting
         * powers.*/
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    /** runs while the OpMode is running (between pressing PLAY and STOP). */
    public void loop() {

        /** Create new variables to store our calculations
         * for what to set the power of the motors. */
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double rearLeftPower = 0;
        double rearRightPower = 0;

        /** Running calculations to determine wheel power:
         * gamepad1 is our controller
         * Left stick y is used for forward/backward movement
         * Left stick x is used for strafing right/left
         * Right stick x is used for turning*/
        frontLeftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        frontRightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        rearLeftPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        rearRightPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;


        /** Apply Power to motors. */
        frontLeft.setPower(frontLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        rearLeft.setPower(rearLeftPower * speedMultiplier);
        rearRight.setPower(rearRightPower * speedMultiplier);

    }

}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class RubberDuckieTele extends OpMode {
    //Define what we have for electronics on the Robot

    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor arm;

    CRServo roller;

    Servo ropeatater;

    double speedMultiplier = 0.5;

    double ropotaterPos = 1;

    public void init() {
        front_left = hardwareMap.dcMotor.get("fl");
        front_right = hardwareMap.dcMotor.get("fr");
        back_left = hardwareMap.dcMotor.get("rl");
        back_right = hardwareMap.dcMotor.get("rr");
        arm = hardwareMap.dcMotor.get("arm");
        roller = hardwareMap.crservo.get("roll");
        ropeatater = hardwareMap.servo.get("rop");
        ropeatater.scaleRange(0.125, 0.875);
        ropeatater.setPosition(ropotaterPos);

        /** Setting all our motors to apply power to stop in place.*/
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void loop() {
        double frontLeftPower = 0;
        double frontrightPower = 0;
        double backrightPower = 0;
        double backleftPower =0;
        double armPower = 0;

        //Create powers to pass to the motors
        frontLeftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        frontrightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        backrightPower = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        backleftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        armPower = -gamepad2.right_stick_y;

        /** Apply Power to motors. */
        front_left.setPower(frontLeftPower * speedMultiplier);
        front_right.setPower(frontrightPower * speedMultiplier);
        back_left.setPower(backrightPower * speedMultiplier);
        back_right.setPower(backleftPower * speedMultiplier);
        arm.setPower(armPower * speedMultiplier);



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
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Park Auto", group="Robot")

public class auto_park extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         front_left   = null;
    private DcMotor         front_right  = null;
    private DcMotor         back_left = null;
    private DcMotor         back_right = null;


    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        front_left = hardwareMap.dcMotor.get("fl");
        front_right = hardwareMap.dcMotor.get("fr");
        back_left = hardwareMap.dcMotor.get("rl");
        back_right = hardwareMap.dcMotor.get("rr");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);



        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        drive_me(0.5, 0.5, 0.5, 0.5, 2);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }
    public void drive_me(double front_right, double back_right, double back_left, double front_left, double seconds){
        this.front_right.setPower(front_right);
        this.back_right.setPower(back_right);
        this.back_left.setPower(back_left);
        this.front_left.setPower(front_left);
        runtime.reset();
        while(opModeIsActive() && (runtime.seconds() < seconds)){

        }
        this.front_right.setPower(0);
        this.back_right.setPower(0);
        this.back_left.setPower(0);
        this.front_left.setPower(0);
    }

}


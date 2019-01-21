package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpMode extends OpMode {

    //actual code for the main robot

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor lift;
    Servo claw;
    Servo idol;


    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("Right");
        leftMotor = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");
        claw = hardwareMap.servo.get("Claw");
        idol = hardwareMap.servo.get("Idol");
        claw.setPosition(0);
        idol.setPosition(-.25);
    }

    @Override
    public void loop() {

        rightMotor.setPower(gamepad1.right_stick_y);

        leftMotor.setPower(-1 * (gamepad1.left_stick_y));

        lift.setPower(-1*(gamepad2.left_stick_y));

        if (gamepad2.a) {
            claw.setPosition(1);
        } else {
            claw.setPosition(0);
        }
        if (gamepad1.a) {
            idol.setPosition(0);
        } else {
            idol.setPosition(-.25);
        }
    }
}
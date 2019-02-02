///package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpMode extends OpMode {

    //actual code for the main robot

    int xPushed = 3;

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor lift;
    DcMotor arm;
    Servo claw;
    Servo idol;
    Servo armClaw;
    CRServo slide;
    CRServo sweeper;

    @Override
    public void init() {

        rightMotor = hardwareMap.dcMotor.get("Right");
        leftMotor = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");
        arm = hardwareMap.dcMotor.get("Arm");
        claw = hardwareMap.servo.get("Claw");
        idol = hardwareMap.servo.get("Idol");
        armClaw = hardwareMap.servo.get("ArmClaw");
        slide = hardwareMap.crservo.get("Slide");
        sweeper = hardwareMap.crservo.get("Sweeper");
        claw.setPosition(0);
        idol.setPosition(-.25);
        armClaw.setPosition(0);
    }
    @Override
    public void loop() {

//        rightMotor.setPower(gamepad1.right_stick_y);
//
//        leftMotor.setPower(-1 * (gamepad1.left_stick_y));

        rightMotor.setPower(gamepad1.left_stick_y);
        leftMotor.setPower(-1* (gamepad1.left_stick_y));

        rightMotor.setPower(-1*(gamepad1.right_stick_x));
        leftMotor.setPower(-1* (gamepad1.right_stick_x));

        lift.setPower(-1*(gamepad2.left_stick_y));
        arm.setPower(gamepad2.right_stick_y);

        slide.setPower(gamepad2.left_trigger);
        slide.setPower(-1 * (gamepad2.right_trigger));



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

        if (gamepad1.dpad_up) {
            armClaw.setPosition(1);
        }
        else if (gamepad1.dpad_down){
            armClaw.setPosition(0);
        }
        if (gamepad1.x) {
            xPushed++;
            sweeper.setPower(1);
            if (xPushed % 2 == 0) {
                sweeper.setPower(0);
                xPushed--;
            }
        }
    }
}

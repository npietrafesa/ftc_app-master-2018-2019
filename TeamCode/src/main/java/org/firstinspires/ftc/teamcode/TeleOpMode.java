package org.firstinspires.ftc.teamcode;

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

    int xPushed = 0;

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor lift;
    DcMotor arm;
    Servo claw;
    Servo idol;
    CRServo armClaw;
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
        armClaw = hardwareMap.crservo.get("ArmClaw");
        slide = hardwareMap.crservo.get("Slide");
        sweeper = hardwareMap.crservo.get("Sweeper");
        claw.setPosition(0);
        idol.setPosition(.9);
 //       slide.setPosition(.5);
    }
    @Override
    public void loop() {
        rightMotor.setPower(gamepad1.right_stick_y);

        leftMotor.setPower(-1 * (gamepad1.left_stick_y));
//
//        rightMotor.setPower(gamepad1.left_stick_y);
//        leftMotor.setPower(-1* (gamepad1.left_stick_y));
//
//        rightMotor.setPower(-1*(gamepad1.right_stick_x));
//        leftMotor.setPower(-1* (gamepad1.right_stick_x));

        lift.setPower(-1*(gamepad2.left_stick_y));
        arm.setPower(gamepad2.right_stick_y);
        if (gamepad2.right_trigger > 0.2) {
            slide.setPower(.2);
        }
        else if (gamepad2.left_trigger > 0.2) {
            slide.setPower(-.2);
        }
        else {
            slide.setPower(0);
        }


        if (gamepad2.a) {
            claw.setPosition(-.5);
        } else {
            claw.setPosition(.9);
        }

        if (gamepad1.a) {
            idol.setPosition(-1);
        } else {
            idol.setPosition(-.25);
        }

        if (gamepad1.dpad_up) {
            armClaw.setPower(-.25);
        }
        else if (gamepad1.dpad_down){
            armClaw.setPower(.25);
        }
        else armClaw.setPower(0);

        if (gamepad2.x) {
            sweeper.setPower(1);
        }
        else {
            sweeper.setPower(0);
        }
        telemetry.addData("Left", leftMotor.getPower());
        telemetry.addData("Right", rightMotor.getPower());
        telemetry.addData("Lift", lift.getPower());
        telemetry.addData("Arm", arm.getPower());
        telemetry.addData("armClaw", armClaw.getPower());
 //       telemetry.addData("Slide", slide.getPosition());
        telemetry.addData("Sweeper", sweeper.getPower());
        telemetry.addData("Claw", claw.getPosition());
        telemetry.addData("Idol", idol.getPosition());
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp
public class safeHalloweenOpMode extends OpMode {

    //For the dummy bot/safe halloween bot

    DcMotor FrontLeft;
    DcMotor BackLeft;
    DcMotor FrontRight;
    DcMotor BackRight;

    @Override
    public void init() {

        FrontRight = hardwareMap.dcMotor.get("Front Left");
        FrontLeft = hardwareMap.dcMotor.get("Front Right");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");

    }

    @Override
    public void loop() {

        FrontLeft.setPower(gamepad1.left_stick_y);
        BackLeft.setPower(gamepad1.left_stick_y);
        FrontRight.setPower(-1 *(gamepad1.right_stick_y));
        BackRight.setPower(gamepad1.right_stick_y);

        telemetry.addData("Left 1", FrontLeft.getPower());
        telemetry.addData("Left 2", BackLeft.getPower());
        telemetry.addData("Right 1", FrontRight.getPower());
        telemetry.addData("Right 2", BackRight.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();

    }
}

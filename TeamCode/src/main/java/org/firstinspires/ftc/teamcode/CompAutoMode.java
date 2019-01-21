package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class CompAutoMode extends LinearOpMode {

    //vars
    private static double encoderTicksPerRevolution = 1120; //NeveRest 40 have 1120 ppr
    private static final double pi = 3.1415;
    private static double wheelDiameter = 4.0; //wheels are 4 inches in diameter
    private static double wheelGearReduction = 1.0; //gears are in a 1:1 ratio, so no change
    private static final double wheelEncoderTicksPerInch = ((encoderTicksPerRevolution * wheelGearReduction) / (wheelDiameter * pi)); //basic circumference equation to find how many encoder ticks are in one inch travelled.
    private static double liftGearDiameter = 1;
    private static double liftGearReduction = .5;
    private static final double liftEncoderTicksPerInch = ((encoderTicksPerRevolution * liftGearReduction) / (liftGearDiameter * pi));
    private static final double degreeTurnArc = (180 * .2618) / (pi * 15); //how many inches the wheel will travel in one degree

    DcMotor rightMotor;
    DcMotor leftMotor;
    DcMotor lift;
    Servo liftClaw;
    Servo idol;

    //methods
    private void changeState(Step newState) {
        state = newState;
    }

    public void resetState() {
        changeState(Step.readyRobot);
    }

    public void readyForNextState() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        lift.setPower(0);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Step getState(Step state) {
        return state;
    }

    public void moveForward(double power, double inches) {
        if (opModeIsActive()) {
            int leftTargetPosition = lift.getCurrentPosition() + (int) (inches * wheelEncoderTicksPerInch);
            leftMotor.setTargetPosition(leftTargetPosition);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setPower(power * .5);
            leftMotor.setPower(power);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive() && leftMotor.isBusy()) {
                telemetry.addData("Left Ticks", leftMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }


    public void turnRobot(double power, double degrees) { //default turns left, set power negative to turn right
        if (opModeIsActive()) {
            int leftTargetPosition = -1 * (leftMotor.getTargetPosition() + (int) (degrees * degreeTurnArc));
            leftMotor.setTargetPosition(leftTargetPosition);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setPower(power * .5);
            leftMotor.setPower(power);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive() && leftMotor.isBusy()) {
                telemetry.addData("Left Ticks", leftMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void lowerLift(double power, double inches) {
        if (opModeIsActive()) {
            int liftTargetPosition = lift.getCurrentPosition() + (int) (inches * liftEncoderTicksPerInch);
            lift.setTargetPosition(liftTargetPosition);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(power); //do not set this above .5, the lift is sensitive

            while (opModeIsActive() && lift.isBusy()) {
                telemetry.addData("Lift Ticks", lift.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void stopRobot() {
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //state machine
    public enum Step {
        readyRobot,
        moveOffLander,
        moveForward,
        turnBot,
        stopRobot
    }

    Step state;

    @Override
    public void runOpMode() {

        //hardware map
        rightMotor = hardwareMap.dcMotor.get("Right");
        leftMotor = hardwareMap.dcMotor.get("Left");
        lift = hardwareMap.dcMotor.get("Lift");
        liftClaw = hardwareMap.servo.get("Claw");
        idol = hardwareMap.servo.get("Idol");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //start encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Encoders Ready");
        telemetry.update();
        liftClaw.setPosition(0);
        idol.setPosition(-.1);

        waitForStart();

        resetState();

        //start of autonomous
        while (opModeIsActive()) {
            switch (state) {
                case readyRobot:
                    readyForNextState();
                    changeState(Step.moveOffLander);
                    break;
                case moveOffLander:
                    lowerLift(.5, 33);
                    liftClaw.setPosition(1);
                    lowerLift(.5, -33);
                    readyForNextState();
                    changeState(Step.moveForward);
                    break;
                case moveForward:
                    moveForward(.6, 18);
                    readyForNextState();
                    changeState(Step.turnBot);
                    break;
                case turnBot:
                    turnRobot(.5, 90);
                    readyForNextState();
                    changeState(Step.stopRobot);
                    break;
                case stopRobot:
                    stopRobot();
                    break;
                default:
                    telemetry.addData("Error", "Something Broke");
                    telemetry.update();
                    break;
            }
//            telemetry.addData("Step", getState(state));
//            telemetry.addData("Ticks Left", leftMotor.getCurrentPosition());
//            telemetry.addData("Ticks Right", rightMotor.getCurrentPosition());
//            telemetry.addData("Ticks Lift", lift.getCurrentPosition());
//            telemetry.update();
        }
    }
}

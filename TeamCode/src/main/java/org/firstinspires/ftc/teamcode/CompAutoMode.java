package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous
public class CompAutoMode extends LinearOpMode {

    //vars
    //encoders
    private static double encoderTicksPerRevolution = 1110; //NeveRest 40 have 1120 ppr (ours have 1110 however)
    private static final double pi = 3.1415;
    private static double wheelDiameter = 4.0; //wheels are 4 inches in diameter
    private static double wheelGearReduction = 1.0; //gears are in a 1:1 ratio, so no change
    private static final double wheelEncoderTicksPerInch = ((encoderTicksPerRevolution * wheelGearReduction) / (wheelDiameter * pi)); //basic circumference equation to find how many encoder ticks are in one inch travelled.
    private static double liftGearDiameter = 1;
    private static double liftGearReduction = .5;
    private static final double liftEncoderTicksPerInch = ((encoderTicksPerRevolution * liftGearReduction) / (liftGearDiameter * pi));
    private static double robotDiameter = 15; //distance in inches between the two wheels
    private static final double degreeTurnArc = (1/360)*(2*pi*(robotDiameter/2))*(wheelEncoderTicksPerInch); //how many inches the wheel will travel in one degree
    //nav
    private static final String VUFORIA_KEY = "AXijc37/////AAAAGR8Zcpk0OkqfpylpmW5pYTAUkEXgtaFwGrLNLr0pw2tXVyNQrJxgegKHKQkDqhX4BfvI/i8II0jj9TXN1WPENa4GY/VYLsafTjuTTSJHctF5OCHh/XH13hEAsGDzW6tFE6SOf8hMHJpKWcv9neasODelhb5jedgNmgYgg9PCOpKPtn66pjIIZoK4XGvj8gH1+sx9WO5Bl3zwDx6IJPDPilKCQ8hhoWyN6g4yck1/ty7dxwx7DDWQ307lSlcg6DINlMaYsR4CIptbTzNE6SSahJPIAL6isd5pYK8iNI2jYyNLRARlTMo1Ps1+KAVUuDo1GI+vvsg/iGCdkjLfZ2qEf415rfqMWgsEAv3dsZs3sdbp";
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private OpenGLMatrix lastLocation = null;
    private boolean navIsFound = false;
    //mineral
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private boolean goldIsFound = false;

    //tf and vuforia
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

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

    public Step getState(Step state) {
        return state;
    }

    public void moveForward(double power, double inches) {
        if (opModeIsActive()) {
            int leftTargetPosition = leftMotor.getCurrentPosition() + (int) (inches * wheelEncoderTicksPerInch);
            int rightTargetPosition = rightMotor.getCurrentPosition() + (int) (inches * wheelEncoderTicksPerInch);
            leftMotor.setTargetPosition(leftTargetPosition);
            rightMotor.setTargetPosition(rightTargetPosition);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setPower(power);
            leftMotor.setPower(power);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()) {
                telemetry.addData("Left Ticks", leftMotor.getCurrentPosition());
                telemetry.addData("Right Ticks", rightMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }


    public void turnRobot(double power, double degrees) { //default turns left, set degrees negative to turn right
        if (opModeIsActive()) {
            int leftTargetPosition = (leftMotor.getCurrentPosition() + (int) (degrees * degreeTurnArc));
            int rightTargetPosition = (rightMotor.getCurrentPosition() + (int) (degrees * degreeTurnArc));
            leftMotor.setTargetPosition(-leftTargetPosition);
            rightMotor.setTargetPosition(rightTargetPosition);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setPower(power);
            leftMotor.setPower(power);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            while (opModeIsActive() && leftMotor.isBusy()) {
                telemetry.addData("Left Ticks", leftMotor.getCurrentPosition());
                telemetry.addData("Right Ticks", rightMotor.getCurrentPosition());
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

    public void lookForGold() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
//            long elapsedTime = System.currentTimeMillis();
            while (opModeIsActive() && !goldIsFound) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    //left
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    telemetry.update();
                                    changeState(Step.goldIsLeft);
                                    goldIsFound = true;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    //right
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    telemetry.update();
                                    changeState(Step.goldIsRight);
                                    goldIsFound = true;
                                }
//                                else if (elapsedTime > 10000 && !goldIsFound) {
//                                    changeState(Step.findNavigationTarget);
//                                    goldIsFound = true;
//                                }
                                else {
                                    //center
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    telemetry.update();
                                    changeState(Step.goldIsCenter);
                                    goldIsFound = true;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void dropIdol() {
        idol.setPosition(0);
    }


    //state machine
    public enum Step {
        readyRobot,
        moveOffLander,
        findGoldMineral,
        goldIsCenter,
        goldIsLeft,
        goldIsRight,
        findNavigationTarget,
        craterSide,
        depotSide,
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
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Encoders Ready");
        telemetry.update();
        liftClaw.setPosition(0);
        idol.setPosition(-.5);

        //vuforia init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraDirection   = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");

        //setting names of nav targets
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        //init nav targets
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        //camera placement
        //PLACEHOLDERS, MAKE SURE TO FIND ACTUAL MEASUREMENTS
        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        //init tensorflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

        telemetry.addData("Status", "Encoders, Vuforia and Tensorflow Ready");
        telemetry.update();

        waitForStart();

        targetsRoverRuckus.activate();
        resetState();

        //start of autonomous
        while (opModeIsActive()) {
            switch (state) {
                case readyRobot:
                    changeState(Step.moveOffLander);
                    break;
                case moveOffLander:
                    lowerLift(.5, 30);
                    liftClaw.setPosition(1);
//                    lowerLift(.5, -30);
                    changeState(Step.findGoldMineral);
                    break;
                case findGoldMineral:
                    lookForGold();
                    break;
                case goldIsLeft:
                    moveForward(.5, 6);
                    turnRobot(.5, 14);
                    moveForward(.5, 24);
                    moveForward(.5, -14);
                    //turnRobot(.5, 7);
                    changeState(Step.findNavigationTarget);
                    break;
                case goldIsRight:
                    moveForward(.5, 6);
                    turnRobot(.5, 7);
                    moveForward(.5, 24);
                    moveForward(.5, -14);
                    turnRobot(-.5, -7);
                    changeState(Step.findNavigationTarget);
                    break;
                case goldIsCenter:
                    moveForward(.5, 26);
                    moveForward(.5, -14);
                    changeState(Step.findNavigationTarget);
                    break;
                case findNavigationTarget:
//                    turnRobot(.5, 20);
//                    moveForward(.5, 36);
                    //start looking for nav target
                    long elapsedTime = System.currentTimeMillis();
                    while (opModeIsActive() && !navIsFound) {
                    // check all the trackable target to see which one (if any) is visible.
                    navIsFound = false;
                    for (VuforiaTrackable trackable : allTrackables) {
                        if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                            telemetry.addData("Visible Target", trackable.getName());
                            navIsFound = true;

                            switch (trackable.getName()) {
                                case "Blue-Rover":
                                    changeState(Step.craterSide);
                                    navIsFound = true;
                                    break;
                                case "Red-Footprint":
                                    changeState(Step.craterSide);
                                    navIsFound = true;
                                    break;
                                case "Front-Craters":
                                    changeState(Step.depotSide);
                                    navIsFound = true;
                                    break;
                                case "Back-Space":
                                    changeState(Step.depotSide);
                                    navIsFound = true;
                                    break;
                            }
//                            if (elapsedTime > 8000 && !navIsFound) {
//                                changeState(Step.stopRobot);
//                            }
                            // getUpdatedRobotLocation() will return null if no new information is available since
                            // the last time that call was made, or if the trackable is not currently visible.
                            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                            if (robotLocationTransform != null) {
                                lastLocation = robotLocationTransform;
                            }
                            break;
                        }
                    }
                    // Provide feedback as to where the robot is located (if we know).
//                    if (navIsFound) {
//                        // express position (translation) of robot in inches.
//                        VectorF translation = lastLocation.getTranslation();
//                        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                                translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//                        // express the rotation of the robot in degrees.
//                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//                        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//                    }
//                    else {
//                        telemetry.addData("Visible Target", "none");
//                    }
                    telemetry.update();
                }
                    break;
                case depotSide:
//                    turnRobot(.5,135);
//                    moveForward(.5, 18);
//                    dropIdol();
                    changeState(Step.stopRobot);
                    break;
                case craterSide:
//                    turnRobot(.5, 45);
//                    moveForward(.5, 18);
//                    dropIdol();
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
        }
    }
}

package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOp.ScoringFunctions.TeleOpScoringFunctions;

@TeleOp
public class FieldOriented2 extends LinearOpMode {
    /*

     * Proportional Integral Derivative Controller

     */

    ElapsedTime scoringTime = new ElapsedTime();

    ElapsedTime waitingTime = new ElapsedTime();

    public DcMotor slide;


    // Elapsed timer class from SDK, please use it, it's epic
    public ElapsedTime timer = new ElapsedTime();

    public Servo door;
    public Servo larm;
    public Servo rarm;
    public CRServo wheel;
    public DcMotor intake;
    public Servo plane;
    boolean isTriggerPressed = false;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private BNO055IMU imu; // Gyro sensor
    private boolean gyroResetRequested = false;


    TeleOpScoringFunctions runMyRobot = new TeleOpScoringFunctions();

    private TeleOpScoringFunctions.robotMachineState targetMacro = TeleOpScoringFunctions.robotMachineState.SLIDE_MIN;


    @Override
    public void runOpMode() {

        slide = hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel = hardwareMap.crservo.get("wheel");
        door = hardwareMap.get(Servo.class, "door");
        larm = hardwareMap.get(Servo.class, "larm");
        rarm = hardwareMap.get(Servo.class, "rarm");
        intake = hardwareMap.dcMotor.get("intake");
        plane = hardwareMap.get(Servo.class, "plane");
        door.setDirection(Servo.Direction.FORWARD);
        larm.setDirection(Servo.Direction.REVERSE);
        rarm.setDirection(Servo.Direction.FORWARD);
        plane.setDirection(Servo.Direction.FORWARD);
        plane.scaleRange(0.0, 1.0);
        larm.scaleRange(0.0, 1.0);
        rarm.scaleRange(0.0, 1.0);
        door.setPosition(0.0);
        wheel.setPower(0.0);
        larm.setPosition(0.0);
        rarm.setPosition(0.0);
        plane.setPosition(0.0);
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the gyro sensor
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.calibrationDataFile = "BNO055IMUCalibration.json"; // Set this to your calibration file
        imuParams.loggingEnabled = true;
        imuParams.loggingTag = "IMU";
        imuParams.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);

        scoringTime.startTime();
        scoringTime.reset();
        waitingTime.startTime();
        waitingTime.reset();

        waitForStart();

        boolean wasdpad_leftPressed = false;
        boolean wasdpad_upPressed = false;
        boolean wasdpad_rightPressed = false;
        boolean wasdpad_downPressed = false;
        boolean wasbPressed = false;
        boolean wasxPressed = false;

        while (opModeIsActive()) {
            if (gamepad1.dpad_left && !wasdpad_leftPressed) {

                //debounce time

                targetMacro = TeleOpScoringFunctions.robotMachineState.SLIDE_MIN;
                //change this

            }
            wasdpad_leftPressed = gamepad1.dpad_left;

            if (gamepad1.dpad_up && !wasdpad_upPressed) {

                waitingTime.reset();
                targetMacro = TeleOpScoringFunctions.robotMachineState.SLIDE_MID;

                /*controlSlides(10000);
                larm.setPosition(0.75);
                rarm.setPosition(0.75);
                sleep(500);
                door.setPosition(1.0);*/
            }
            wasdpad_upPressed = gamepad1.dpad_up;

            if (gamepad1.dpad_right && !wasdpad_rightPressed) {


                waitingTime.reset();
                targetMacro = TeleOpScoringFunctions.robotMachineState.SLIDE_MAX;

               /* controlSlides(15000);
                larm.setPosition(0.75);
                rarm.setPosition(0.75);
                sleep(500);
                door.setPosition(1.0);*/
            }
            wasdpad_rightPressed = gamepad1.dpad_right;

            if (gamepad1.dpad_down && !wasdpad_downPressed) {


                waitingTime.reset();
                targetMacro = TeleOpScoringFunctions.robotMachineState.SLIDE_ZERO;


                /*larm.setPosition(0.0);
                rarm.setPosition(0.0);
                door.setPosition(0.0);
                sleep(2000);
                controlSlides(0);*/
            }
            wasdpad_downPressed = gamepad1.dpad_down;

            if (gamepad1.left_trigger > .5) {

                intake.setPower(1);
                wheel.setPower(-2);
                isTriggerPressed = true;

            } else {

                if (isTriggerPressed) {
                    intake.setPower(0);
                    wheel.setPower(0);
                    isTriggerPressed = false;
                }
            }
            if (gamepad1.right_trigger > .5) {
                intake.setPower(-1);
                wheel.setPower(3);
                isTriggerPressed = true;
            } else {
                if (isTriggerPressed) {
                    intake.setPower(0);
                    wheel.setPower(0);
                    isTriggerPressed = false;
                }
            }
            if (gamepad1.left_bumper) {
                door.setPosition(0.0);

            } else {
                telemetry.addData("testing", "true");
            }
            if (gamepad1.right_bumper) {
                door.setPosition(1.0);

            } else {
                telemetry.addData("testing2", "true");
            }
            if (gamepad1.b && !wasbPressed) {
                plane.setPosition(1.0);
            }
            wasbPressed = gamepad1.b;
            if (gamepad1.x && !wasxPressed) {
                plane.setPosition(0.0);
            }
        }


        runMyRobot.doThisThingy(slide, larm, rarm, door, scoringTime, waitingTime, targetMacro);
        //this is where you tell uit what to do
//

        // Get joystick inputs from the gamepad
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        // reset gyro button
        if (gamepad1.a) {
            gyroResetRequested = true;
        }

        // Perform gyro reset if requestede
        if (gyroResetRequested) {
            resetGyro();
            gyroResetRequested = false; // Reset the request flag
        }

        // Get the robot's heading from the gyro sensor
        double heading = getHeading();
        // Calculate the joystick inputs in the field-oriented frame of reference
        double fieldDrive = drive * Math.cos(Math.toRadians(heading)) - strafe * Math.sin(Math.toRadians(heading));
        double fieldStrafe = drive * Math.sin(Math.toRadians(heading)) + strafe * Math.cos(Math.toRadians(heading));
        // Calculate motor powers for mecanum drive

        double frontLeftPower = fieldDrive + fieldStrafe + rotate;
        double frontRightPower = fieldDrive - fieldStrafe - rotate;
        double rearLeftPower = fieldDrive - fieldStrafe + rotate;
        double rearRightPower = fieldDrive + fieldStrafe - rotate;

        // Ensure motor powers are within the valid range of -1 to 1
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        rearLeftPower = Range.clip(rearLeftPower, -1.0, 1.0);
        rearRightPower = Range.clip(rearRightPower, -1.0, 1.0);

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearLeftMotor.setPower(rearLeftPower);
        rearRightMotor.setPower(rearRightPower);

        // Display motor powers on telemetry (optional)
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Rear Left Power", rearLeftPower);
        telemetry.addData("Rear Right Power", rearRightPower);
        telemetry.update();

        // Update telemetry and control motors
        telemetry.addData("Gyro Heading", heading);
        telemetry.update();
    }

    /*public void slidesmin(){
        controlSlides(5000);
        larm.setPosition(0.75);
        rarm.setPosition(0.75);
    }
    public void slidesmid(){
        controlSlides(20000);
        larm.setPosition(0.75);
        rarm.setPosition(0.75);
    }
    public void slidesmax(){
        controlSlides(15000);
        larm.setPosition(0.75);
        rarm.setPosition(0.75);
    }*/

    private double getHeading() {
        // Get the robot's heading from the gyro sensor
        return imu.getAngularOrientation().firstAngle;
    }
    private void resetGyro() {
        // Reset the gyro orientation to its initial state
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // Optional: Load a calibration file if available
        imu.initialize(parameters);
    }
}

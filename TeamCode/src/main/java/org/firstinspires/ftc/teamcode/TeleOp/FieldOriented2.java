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

@TeleOp
public class FieldOriented2 extends LinearOpMode {
    /*

     * Proportional Integral Derivative Controller

     */
    public DcMotor slide;
    public double Kp = 0.02;
    public double Ki = 0;
    public double Kd = 0;
    public double integralSum = 0;
    public double lastError = 0;
    public final int CUTOFF = 20;

    // Elapsed timer class from SDK, please use it, it's epic
    public ElapsedTime timer = new ElapsedTime();

    public Servo door;
    public Servo larm;
    public Servo rarm;
    public CRServo wheel;
    public DcMotor intake;
    boolean isTriggerPressed = false;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor rearLeftMotor;
    private DcMotor rearRightMotor;
    private BNO055IMU imu; // Gyro sensor
    private boolean gyroResetRequested = false;

    @Override
    public void runOpMode() {

        slide = hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheel = hardwareMap.crservo.get("wheel");
        door = hardwareMap.get(Servo.class, "door");
        larm = hardwareMap.get(Servo.class, "larm");
        rarm = hardwareMap.get(Servo.class, "rarm");
        intake =hardwareMap.dcMotor.get("intake");
        door.setDirection(Servo.Direction.FORWARD);
        larm.setDirection(Servo.Direction.REVERSE);
        rarm.setDirection(Servo.Direction.FORWARD);
        larm.scaleRange(0.0, 1.0);
        rarm.scaleRange(0.0, 1.0);
        door.setPosition(0.0);
        wheel.setPower(0);
        larm.setPosition(0.0);
        rarm.setPosition(0.0);
        frontLeftMotor = hardwareMap.dcMotor.get("FL");
        frontRightMotor = hardwareMap.dcMotor.get("FR");
        rearLeftMotor = hardwareMap.dcMotor.get("BL");
        rearRightMotor = hardwareMap.dcMotor.get("BR");


        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        waitForStart();

        boolean wasYPressed = false;
        boolean wasBPressed = false;
        while(opModeIsActive()) {
            if(gamepad1.dpad_left && !wasYPressed){
                controlSlides(400);
                larm.setPosition(0.75);
                rarm.setPosition(0.75);
            }
            wasYPressed = gamepad1.dpad_left;

            if(gamepad1.dpad_up && !wasYPressed){
                controlSlides(1200);
                larm.setPosition(0.75);
                rarm.setPosition(0.75);
            }
            wasYPressed = gamepad1.dpad_up;

            if(gamepad1.dpad_right && !wasYPressed){
                controlSlides(1400);
                larm.setPosition(0.75);
                rarm.setPosition(0.75);
            }
            wasYPressed = gamepad1.dpad_right;

            if (gamepad1.dpad_down && !wasBPressed){
                larm.setPosition(0.0);
                rarm.setPosition(0.0);
                sleep(2000);
                controlSlides(0);
            }
            wasBPressed = gamepad1.dpad_down;

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

            }


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
        sleep(2000);
    }

    public void controlSlides(int setpoint) {
        while (opModeIsActive() && Math.abs(slide.getCurrentPosition() - setpoint) > CUTOFF) {

            // obtain the encoder position
            int encoderPosition = slide.getCurrentPosition();
            // calculate the error
            double error = setpoint - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
            out = Range.clip(out, -1, 1);

            slide.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();
        }

        slide.setPower(0);

    }
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

package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime; // Import ElapsedTime



import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(group = "drive")
public class AutoLeftBlue extends LinearOpMode {
    public OpenCvWebcam webcam1 = null;
    public ElapsedTime elapsedTime = new ElapsedTime(); // Add ElapsedTime to track time
    public double totalLeftAvg = 0;
    public double totalRightAvg = 0;
    public double left = 0;
    public double right = 0;
    public int frameCount = 0;
    public int zone = 0;
    public ExamplePipeline examplePipeline;
    public Servo AutoFinger;
    DcMotor intake;
    public Servo door;
    public Servo larm;
    public Servo rarm;
    public CRServo wheel;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    private double headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  FLSpeed       = 0;
    private double  BLSpeed       = 0;
    private double  FRSpeed       = 0;
    private double  BRSpeed       = 0;

    private int  FLTarget  = 0;
    private int  BLTarget  = 0;
    private int  FRTarget   = 0;
    private int  BRTarget   = 0;

    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.6;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.6;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.04;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.04;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
// Initialize the drive system variables.
        FL  = hardwareMap.get(DcMotor.class, "FL");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        FR  = hardwareMap.get(DcMotor.class, "FR");
        BR  = hardwareMap.get(DcMotor.class, "BR");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)

        telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
        //telemetry.update();

        // Set the encoders for closed loop speed control, and reset the heading.
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();
        // Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        //  drive.setPoseEstimate(startPose);
        AutoFinger = hardwareMap.get(Servo.class, "door");
        AutoFinger.setPosition(0.0);
        AutoFinger.setDirection(Servo.Direction.FORWARD);

        //intake = hardwareMap.dcMotor.get("intake");
        //wheel = hardwareMap.crservo.get("wheel");
        //door = hardwareMap.get(Servo.class, "door");
        //larm = hardwareMap.get(Servo.class, "larm");
        //rarm = hardwareMap.get(Servo.class, "rarm");

        //door.setDirection(Servo.Direction.FORWARD);
        //larm.setDirection(Servo.Direction.REVERSE);
        //rarm.setDirection(Servo.Direction.FORWARD);
        //larm.scaleRange(0.0, 1.0);
        //rarm.scaleRange(0.0, 1.0);
        //door.setPosition(0.0);
        //wheel.setPower(0);
        //larm.setPosition(0.0);
        //rarm.setPosition(0.0);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        examplePipeline = new ExamplePipeline();
        webcam1.setPipeline(examplePipeline);

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }//TODO: adjust width and height baced on specific camera

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.addLine("Waiting to start");
        //  telemetry.update();
        waitForStart();
        // Run for 10 seconds
        telemetry.addLine("started");
        // telemetry.update();
      /*  for (int i = 0; i < 200; i++) { // 200 iterations * 50 milliseconds = 10 seconds
            telemetry.addLine("Measuring Camara stream");
            telemetry.update();
            // Process frames and accumulate color values
            totalLeftAvg += examplePipeline.leftavgfin;
            totalRightAvg += examplePipeline.rightavgfin;
            frameCount++;

            // Sleep for 50 milliseconds
            sleep(50);
        }*/

        left = examplePipeline.leftavgfin;
        right = examplePipeline.rightavgfin;

        telemetry.addLine("done computing");
        //telemetry.update();
        // Average color values over ten seconds
        double averageLeft = left;//totalLeftAvg / frameCount;
        double averageRight = right;//totalRightAvg / frameCount;

        telemetry.addLine("Returning Values");
        // telemetry.update();
        // Use the average values to determine autonomous steps
        if (left > right && (Math.abs(left - right)) >= 1.5) {
            zone = 1;
            //left
            telemetry.addData("Zone", zone);
            telemetry.addData("Average Left Value", averageLeft);
            telemetry.addData("Average Right Value", averageRight);
            telemetry.update();

            //write your Autonomous specific instructions for this spike mark zone
            AutoFinger.setPosition(0.7);
            sleep(800);
            driveStraight(DRIVE_SPEED, -27, 0);
            sleep(800);
            driveStrafe(DRIVE_SPEED, 8, 0);
            sleep(800);
            AutoFinger.setPosition(0.0);
            driveStraight(DRIVE_SPEED, 25, 0);
            sleep(800);
            turnToHeading(DRIVE_SPEED, 85);
            sleep(1000);
            driveStraight(DRIVE_SPEED, -30, 85);
            driveStrafe(DRIVE_SPEED, -7,  90);
            driveStraight(DRIVE_SPEED, -10, 90);
            driveStrafe(DRIVE_SPEED, -25, 90);
            turnToHeading(TURN_SPEED, 180);
            sleep(700);
            driveStraight(DRIVE_SPEED, -10, 180);

        } else if (left < right && (Math.abs(left - right)) >= 1.5) {
            zone = 2;
            //middle
            telemetry.addData("Zone", zone);
            telemetry.addData("Average Left Value", averageLeft);
            telemetry.addData("Average Right Value", averageRight);
            // telemetry.update();

            //write your Autonomous specific instructions for this spike mark zone
            AutoFinger.setPosition(0.7);
            sleep(800);
            driveStraight(DRIVE_SPEED, -31, 0);
            driveStrafe(DRIVE_SPEED, -3, 0);
            sleep(800);
            AutoFinger.setPosition(0.0);
            driveStraight(DRIVE_SPEED, 28, 0);
            sleep(800);
            turnToHeading(DRIVE_SPEED, 85);
            sleep(800);
            driveStraight(DRIVE_SPEED, -25, 85);
            driveStrafe(DRIVE_SPEED, -30,  90);
            driveStraight(DRIVE_SPEED, -14, 90);
            driveStrafe(DRIVE_SPEED, -29, 90);
            turnToHeading(TURN_SPEED, 180);

        } else {
            zone = 3;
            //right
            telemetry.addData("Zone", zone);
            //    telemetry.update();

            //write your Autonomous specific instructions for this spike mark zone
            AutoFinger.setPosition(0.7);
            sleep(800);
            driveStraight(DRIVE_SPEED, -27, 0);
            sleep(800);
            turnToHeading(DRIVE_SPEED, 85);
            sleep(1000);
            driveStraight(DRIVE_SPEED, -7, 85);
            AutoFinger.setPosition(0.0);
            driveStraight(DRIVE_SPEED, 15, 90);
            turnToHeading(TURN_SPEED, 270);
            sleep(700);
            driveStraight(DRIVE_SPEED, -28, 270);
            driveStrafe(DRIVE_SPEED, 19,  270);
            driveStraight(DRIVE_SPEED, -8, 270);
            driveStrafe(DRIVE_SPEED, 33, 270);
            turnToHeading(TURN_SPEED, -180);



        }


        telemetry.addData("Path", "Complete");
        // telemetry.update();
        sleep(4000);





    }



    class ExamplePipeline extends OpenCvPipeline {

        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();

        Scalar rectColor = new Scalar(0.0, 0.0, 255.0);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
//specific square size
            Rect leftRect = new Rect(350, 280, 280, 230);
            Rect rightRect = new Rect(940, 270, 280, 250);//middle is 640
            //changing the above 800 to 640

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, leftRect, rectColor, 20);
            Imgproc.rectangle(outPut, rightRect, rectColor, 20);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);

            Core.extractChannel(leftCrop, leftCrop, 2);
            Core.extractChannel(rightCrop, rightCrop, 2);
            //coi 2 is blue

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

     /*       telemetry.addLine("pipeline running");
            telemetry.addData("LeftValue", leftavgfin);
            telemetry.addData("RightValue", rightavgfin);
*/
            return outPut;
        }
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            FLTarget = FL.getCurrentPosition() + moveCounts;
            BLTarget = BL.getCurrentPosition() + moveCounts;
            FRTarget = FR.getCurrentPosition() + moveCounts;
            BRTarget = BR.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FL.setTargetPosition(FLTarget);
            BL.setTargetPosition(BLTarget);
            FR.setTargetPosition(FRTarget);
            BR.setTargetPosition(BRTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveStrafe(double maxDriveSpeed,
                            double distance,
                            double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            FLTarget = FL.getCurrentPosition() + moveCounts;
            BLTarget = BL.getCurrentPosition() - moveCounts;
            FRTarget = FR.getCurrentPosition() - moveCounts;
            BRTarget = BR.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            FL.setTargetPosition(FLTarget);
            BL.setTargetPosition(BLTarget);
            FR.setTargetPosition(FRTarget);
            BR.setTargetPosition(BRTarget);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (opModeIsActive() &&
                    (FL.isBusy() && BL.isBusy() && FR.isBusy() && BR.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.

            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (FR/BR/FL/BL) requests,
     * combines them, and applies the appropriate speed commands to the FL, BL, FR, and BR wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        FLSpeed = drive - turn;
        BLSpeed = drive - turn;
        FRSpeed = drive + turn;
        BRSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(FLSpeed), Math.abs(BLSpeed)), Math.max(Math.abs(FRSpeed), Math.abs(BRSpeed)));
        if (max > 1.0)
        {
            FLSpeed /= max;
            BLSpeed /= max;
            FRSpeed /= max;
            BRSpeed /= max;
        }

        FL.setPower(FLSpeed);
        BL.setPower(BLSpeed);
        FR.setPower(FRSpeed);
        BR.setPower(BRSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos FL:BL:FR:BR",  "%7d:%7d",      FLTarget, BLTarget, FRTarget, BRTarget);
            telemetry.addData("Actual Pos FL:BL:FR:BR",  "%7d:%7d",      FL.getCurrentPosition(),BL.getCurrentPosition(),FR.getCurrentPosition(),
                    BR.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds FL:BL:FR:BR", "%5.2f : %5.2f", FLSpeed, BLSpeed, FRSpeed, BRSpeed);
        //    telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


}
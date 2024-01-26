package org.firstinspires.ftc.teamcode.TeleOp.ScoringFunctions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class AnnaPIDslides  {

    public double Kp = 0.02;
    public double Ki = 0;
    public double Kd = 0;
    public double integralSum = 0;
    public double lastError = 0;
    public final int CUTOFF = 20;


    public int targetInputSlidesValue = 0;

    public void magicPID(DcMotor slide, int setpoint, ElapsedTime timer) {

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
}
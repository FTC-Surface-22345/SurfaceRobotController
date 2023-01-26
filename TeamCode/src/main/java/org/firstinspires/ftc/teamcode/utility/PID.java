package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    public static double Kp = 0; //
    public static double Ki = 0; //
    public static double Kd = 0; //

    double target;
    double error;
    double derivative;
    double lastError;

    double integralSum = 0;

    double out;
    ElapsedTime timer = new ElapsedTime();

    public void init(ElapsedTime newTimer, double p, double i, double d) {
        timer = newTimer;
        lastError = 0;

        Kp = p;
        Ki = i;
        Kd = d;
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }

    public double update(int motorPos) {
        error = target - motorPos;

        derivative = (error - lastError) / timer.seconds();

        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        timer.reset();

        return out;
    }
}

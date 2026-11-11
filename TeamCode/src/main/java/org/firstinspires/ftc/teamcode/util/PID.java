package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup
 * This is specifically made for auto turning
 */
public class PID {
    private final double m_Kp;
    private final double m_Ki;
    private final double m_Kd;

    private double lastPosition;
    private double lastReference;
    private double previousFilterEstimate;
    private double integralSum = 0;
    private double lastError = 0;
    // max out integral sum
    private final double maxIntegralSum = 100;
    private double lastTarget;

    private final ElapsedTime timer;
    private double lastTime;
    private final ElapsedTime calcTimer;

    public PID(double Kp, double Ki, double Kd) {
        this.m_Kp = Kp;
        this.m_Ki = Ki;
        this.m_Kd = Kd;
        timer = new ElapsedTime();
        lastTime = timer.seconds();
        calcTimer = new ElapsedTime();
    }

    public double calculate(double reference, double position) {
        double deltaTime = calcTimer.seconds();
        calcTimer.reset();
        if (deltaTime <= 0.0001) {
            return 0;
        }

        double error = ((position - reference + 180) % 360 + 360) % 360 - 180;
        boolean setPointIsNotReached = !(Math.abs(error) <= 0.75);

        if (setPointIsNotReached) {
            double headingChange = ((((reference - lastReference) + 180) % 360 + 360) % 360 - 180) / deltaTime;
            lastReference = reference;

            // filter out high frequency noise to increase derivative performance
            // Low-pass filter derivative
            double a = 0.8;
            double derivative = (a * previousFilterEstimate) + ((1 - a) * -headingChange);
            previousFilterEstimate = derivative;

            // sum of all error over time
            integralSum += error * deltaTime;

            integralSum = Math.max(
                    -maxIntegralSum,
                    Math.min(maxIntegralSum, integralSum)
            );

            // reset integral sum upon setpoint changes
            if (position != lastPosition) {
                integralSum = 0;
            }

            lastError = error;
            lastPosition = position;

            double power = (m_Kp * error) + (m_Ki * integralSum) + (m_Kd * derivative);

            return Math.max(-0.75, Math.min(0.75, power));
        }

        integralSum = 0;
        lastError = error;
        previousFilterEstimate = 0;

        return 0;
    }

    public double calc(double current, double target) {
        double error = current - target;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        double time = timer.seconds();
        double dt = time - lastTime;
        lastTime = time;

        if (Math.abs(error) >= 0.5) {
            // Proportional term
            double p = m_Kp * error;

            // Integral term
            integralSum += error * dt;
            double i = m_Ki * integralSum;
            integralSum = Math.max(
                    -maxIntegralSum,
                    Math.min(maxIntegralSum, integralSum)
            );

            // Derivative term
            double derivative = (0.8 * previousFilterEstimate) + ((1 - 0.8) * ((error - lastError) / dt));
            previousFilterEstimate = derivative;
            double d = m_Kd * derivative;

            if (target != lastTarget) {
                integralSum = 0;
            }

            lastError = error;
            lastTarget = target;

            // Total output power/speed adjustment
            double output = p + i + d;
            return Range.clip(output, -0.75, 0.75);
        } else {
            integralSum = 0;
            lastError = error;
            previousFilterEstimate = 0;

            return 0;
        }
    }
}

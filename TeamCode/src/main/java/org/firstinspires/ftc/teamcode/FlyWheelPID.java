package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class FlyWheelPID {

    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */

    public static double Kp;
    public static double Ki;
    public static double Kd;

    public FlyWheelPID(double Kp, double Ki, double Kd) {
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */

    double integralSum = 0;

    double lastError = 0;

    ElapsedTime timer = new ElapsedTime();

    public double update(double target, double state) {
        // obtain the encoder position

        // calculate the error
        double error = target - state;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        lastError = error;
        timer.reset();

        return((Kp * error) + (Ki * integralSum) + (Kd * derivative));

    }
}
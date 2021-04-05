package org.firstinspires.ftc.teamcode.utilclasses;

public class PIDController {

    double target = 0.0;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double time_diff = 0;
    double pos_diff = 0;
    double prev_time = 0;
    double time = 0;
    double prev_error = 0;
    double error = 0;
    double prev_pos = 0;
    double pos = 0;
    double prev_velocity = 0;
    double velocity = 0;

    double integral_error = 0;

    double lim = 0.2;
    double max_input = 200.0;

    public PIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double input, double current_time){
        // Set current time and position (encoder poss)
        time = current_time;
        pos = input;

        // Set differences between current time/pos and previous time/pos
        time_diff = time - prev_time;
        pos_diff = pos - prev_pos;

        // Get current speed in pos units / seconds
        velocity = (pos_diff) / (time_diff);

        // Set error (desired position - current position)
        error = (target - pos);

        // Add to the bound integral error (error is multiplied by time between loops)
        if(integral_error < lim / 2){
            integral_error += error * time_diff;
        }

        // Create and set the P, I, and D terms
        double p_term, i_term, d_term;

        p_term = kP * error;
        i_term = kI * integral_error;
        d_term = kD * velocity;

        // Set the motor power to the terms' sum
        double output = p_term + i_term + d_term;

        // Bound the power value with the maximum and minimum accepted by the motor (1.0 & -1.0)
        output /= max_input;
        if(output > 1.0){
            output = 1.0;
        } else if(output < -1.0){
            output = -1.0;
        }

        // Set up the values for next time
        prev_time = time;
        prev_pos = pos;
        prev_velocity = velocity;
        prev_error = error;

        return output;
    }

    public void setP(double kP){ this.kP = kP;}

    public void setI(double kI){ this.kI = kI;}

    public void setD(double kD){ this.kD = kD;}

    public void setMaxInputs(double max_input, double min_input){

    }
}

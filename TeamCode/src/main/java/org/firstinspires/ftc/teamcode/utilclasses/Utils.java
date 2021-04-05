package org.firstinspires.ftc.teamcode.utilclasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Utils {

    static double ppr = 28;

    /**
     * @param rpm rotations per minute
     * @return ticks per second
     */
    public static double rpm_to_tps(double rpm){
        return (ppr * rpm) / (60);
    }

    /**
     * @param tps ticks per second
     * @return rotations per minute
     */
    public static double tps_to_rpm(double tps){
        return (60 * tps) / (ppr);
    }

    /**
     * Pass PIDF coefficients to a DcMotorEx motor
     * @param motor Motor to pass coefficients to
     * @param coefficients Coefficients to feed to the motor
     * @param battery Battery voltage sensor used to calculate f coefficient
     */
    public static void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients, VoltageSensor battery) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / battery.getVoltage()));
    }
}
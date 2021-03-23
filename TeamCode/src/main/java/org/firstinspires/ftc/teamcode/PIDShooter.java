package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


@TeleOp
@Config
public class PIDShooter extends LinearOpMode {
    DcMotorEx shooterMotor;
    ElapsedTime timer;
    TestOPMode a_g = new TestOPMode();

    double target = 1000.0;
    public static PIDFCoefficients coefficients;

    double time_diff = 0;
    double tick_diff = 0;
    double prev_time = 0;
    double time = 0;
    double prev_error = 0;
    double error = 0;
    double prev_ticks = 0;
    double ticks = 0;
    double prev_speed = 0;
    double speed = 0;
    double[] speeds = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    double integral_error = 0;

    double ppr = 28.0;
    double power = 0.0;
    double lim = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        coefficients = new PIDFCoefficients(0.25, 0.0, 0.0, 0.0);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        FtcDashboard dashboard = FtcDashboard.getInstance();

        target = rpm_to_tps(target);

        waitForStart();
        timer.reset();
        while(opModeIsActive()){
            shooterMotor.setVelocity(target);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Velocity", tps_to_rpm(shooterMotor.getVelocity()));
            packet.put("Target", tps_to_rpm(target));
            dashboard.sendTelemetryPacket(packet);

            /*
            // Set current time and position (encoder ticks)
            time = timer.time();
            ticks = shooterMotor.getCurrentPosition();

            // Set differences between current time/ticks and previous time/ticks
            time_diff = time - prev_time;
            tick_diff = ticks - prev_ticks;

            // Get current rpm in ticks / seconds
            speed = (tick_diff) / (time_diff);

            // Update speed list and average speed
            double[] i = {speed, speeds[0], speeds[1], speeds[2], speeds[3], speeds[4],
                    speeds[5], speeds[6], speeds[7], speeds[8], speeds[9]};
            speeds = i;
            double avg_speed = a_g.get_average(speeds);

            // Set error (desired speed - current speed)
            error = (target - avg_speed);

            // Add more weight to positive error
            if(error > 0.0){
                error *= 1.5;
            }

            // Add to the bound integral error (error is multiplied by time between loops)
            if(integral_error < lim / 2){
                integral_error += error * time_diff;
            }

            // Calculate acceleration for derivative term
            double acceleration = (avg_speed - prev_speed) / (time_diff);

            // Create and set the P, I, and D terms
            double p_term, i_term, d_term;

            p_term = kP * error;
            i_term = kI * integral_error;
            d_term = kD * acceleration;

            // Set the motor power to the terms' sum
            power = p_term + i_term + d_term;

            // Bound the power value with the maximum and minimum accepted by the motor (1.0 & -1.0)
            if(power > lim){
                power = lim;
            } else if(power < 0){
                power = 0;
            }

            shooterMotor.setPower(power);

            telemetry.addData("Power: ", power);
            telemetry.addData("Speed: ", avg_speed);
            telemetry.addData("Error: ", error);
            telemetry.addData("Raw Speed", speed);
            telemetry.update();

            // Send error, target, and speed to the FTCDashboard
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Speed", tps_to_rpm(avg_speed));
            packet.put("Target", tps_to_rpm(target));
            packet.put("Error", tps_to_rpm(error));
            packet.put("Raw speed", tps_to_rpm(speed));
            dashboard.sendTelemetryPacket(packet);

            // Set up the values for next time
            prev_time = time;
            prev_ticks = ticks;
            prev_speed = avg_speed;
            prev_error = error;
             */
        }
        shooterMotor.setPower(0.0);
    }

    /**
     * @param rpm rotations per minute
     * @return ticks per second
     */
    public double rpm_to_tps(double rpm){
        return (ppr * rpm) / (60);
    }

    /**
     * @param tps ticks per second 89
     * @return rotations per minute
     */
    public double tps_to_rpm(double tps){
        return (60 * tps) / (ppr);
    }

    /**
     *
     */
    private void setPIDCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f));
    }

}

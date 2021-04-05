package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
@Config
public class PIDShooter extends LinearOpMode {
    DcMotorEx shooterMotor;

    private VoltageSensor batteryVoltageSensor;

    public static double target = 2500.0;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(80, 0.35, 2.4, 12.5);

    double lastKp = coefficients.p;
    double lastKi = coefficients.i;
    double lastKd = coefficients.d;
    double lastKf = coefficients.f;

    double ppr = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooterMotor.setMotorType(motorConfigurationType);

        shooterMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while(opModeIsActive()){
            shooterMotor.setVelocity(rpm_to_tps(target));

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Velocity", shooterMotor.getVelocity());
            packet.put("Target", rpm_to_tps(target));
            packet.put("Upper Bound", rpm_to_tps(target) * 1.15);
            packet.put("Lower Bound", 0);
            dashboard.sendTelemetryPacket(packet);

            if (lastKp != coefficients.p || lastKi != coefficients.i || lastKd != coefficients.d || lastKf != coefficients.f) {
                shooterMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);

                lastKp = coefficients.p;
                lastKi = coefficients.i;
                lastKd = coefficients.d;
                lastKf = coefficients.f;
            }

            telemetry.addData("Target: ", tps_to_rpm(target));
            telemetry.update();
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
     * @param motor Motor to pass coefficients to
     * @param coefficients Coefficients to feed to the motor
     */
    private void setPIDCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()));
    }

}

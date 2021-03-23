package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDShooter;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestOPMode extends LinearOpMode {
    DcMotor shooterMotor;
    ElapsedTime timer;

    double time_diff = 0;
    double tick_diff = 0;
    double prev_time = 0;
    double time = 0;
    double prev_ticks = 0;
    double ticks = 0;
    double speed = 0;
    double prev_speed = 0;
    double[] speeds = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    double ppr = 28.0;


    @Override
    public void runOpMode() throws InterruptedException{
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        waitForStart();

        if (isStopRequested()) return;

        timer.reset();
        while (opModeIsActive()) {
            // Set current time and position (encoder ticks)
            time = timer.time();
            ticks = shooterMotor.getCurrentPosition();

            // Set differences between current time/ticks and previous time/ticks
            time_diff = time - prev_time;
            tick_diff = ticks - prev_ticks;

            // Get current rpm in ticks / seconds
            speed = (tick_diff) / (time_diff);

            double[] i = {speed, speeds[0], speeds[1], speeds[2], speeds[3], speeds[4],
                          speeds[5], speeds[6], speeds[7], speeds[8], speeds[9]};
            speeds = i;

            double avg_speed = get_average(speeds);

            shooterMotor.setPower(0.1 * gamepad1.left_stick_y);

            // Send error, target, and speed to the FTCDashboard
            // See at 192.168.43.1:8080/dash
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Speed", tps_to_rpm(speed));
            packet.put("Average of 10", tps_to_rpm(avg_speed));
            packet.put("Time between loops", time_diff);
            packet.put("Pos", ticks);
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("Speed:", tps_to_rpm(speed));
            telemetry.addData("Average of 10: ", tps_to_rpm(avg_speed));
            telemetry.addData("Time between loops: ", time_diff);
            telemetry.update();

            // Set up the values for next time
            prev_time = time;
            prev_ticks = ticks;
        }
        shooterMotor.setPower(0.0);
    }


    /**
     * @param tps ticks per second 89
     * @return rotations per minute
     */
    public double tps_to_rpm(double tps){
        return (60 * tps) / (ppr);
    }

    /**
     * @param values list of speed values
     * @return average speed
     */
    public static double get_average(double[] values){
        double num_of_items = 0;
        double total = 0;
        for(double mem : values){
            total += mem;
            num_of_items ++;
        }
        return total / num_of_items;
    }
}

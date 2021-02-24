package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class PIDShooter extends LinearOpMode {
    DcMotor shooterMotor;
    ElapsedTime timer;
    double timeDiff;
    double countDiff;
    double target = 300.0;
    double kP = 0.5;
    double startTicks, endTicks;
    double startTime, endTime;
    double ppr = 537.6;
    double rpm;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();
        timer.reset();
        while(opModeIsActive()){
            startTime = timer.time();
            startTicks = shooterMotor.getCurrentPosition();

            telemetry.addData("Start Time: ", startTime);
            telemetry.addData("Start Ticks: ", startTicks);

            endTime = timer.time();
            endTicks = shooterMotor.getCurrentPosition();

            telemetry.addData("End Time: ", endTime);
            telemetry.addData("End Ticks: ", endTicks);

            timeDiff = endTime - startTime;
            countDiff = endTicks - startTicks;

            rpm = ((countDiff / timeDiff) * 60000) / ppr;
            telemetry.addData("RPM: ", rpm);

            power = kP * ((target - rpm) / target);
            if(power >= 1.0){
                power = 1.0;
            } else if(power <= -1.0){
                power = -1.0;
            }
            telemetry.addData("Power: ", power);

            telemetry.update();

            shooterMotor.setPower(power);
        }

        shooterMotor.setPower(0.0);
    }
}

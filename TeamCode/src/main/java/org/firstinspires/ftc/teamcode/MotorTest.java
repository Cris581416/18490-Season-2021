package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotor shooterMotor;
    ElapsedTime timer;
    double power;
    double timeDiff;
    double countDiff;
    double startTicks, endTicks;
    double startTime, endTime;
    double ppr = 537.6;
    double rpm;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        timer.reset();

        while(opModeIsActive()){
            startTime = timer.milliseconds();
            startTicks = shooterMotor.getCurrentPosition();

            endTime = timer.milliseconds();
            endTicks = shooterMotor.getCurrentPosition();

            timeDiff = endTime - startTime;
            countDiff = endTicks - startTicks;

            rpm = ((countDiff / timeDiff) * 60000) / ppr;

            power = gamepad1.left_stick_y;

            shooterMotor.setPower(power);

            telemetry.addData("Start Time: ", startTime);
            telemetry.addData("Start Ticks: ", startTicks);
            telemetry.addData("End Time: ", endTime);
            telemetry.addData("End Ticks: ", endTicks);
            telemetry.addData("RPM: ", rpm);
            telemetry.addData("Power: ", power);
            telemetry.addData("Time: ", timer.milliseconds());
            telemetry.update();
        }
    }
}

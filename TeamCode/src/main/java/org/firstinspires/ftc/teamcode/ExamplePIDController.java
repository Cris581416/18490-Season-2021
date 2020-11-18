package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ExamplePIDController extends LinearOpMode {
    DcMotor motor;
    ElapsedTime timer;

    double kP = 0.2;
    double kI = 0.0001;
    double kD = 0.001;

    double target = 90;
    double error;

    double pOutput = 0;
    double iOutput = 0;
    double dOutput = 0;
    double power;

    double errorSum = 0;

    double position, lastPosition = 0;
    double time, lastTime, deltaT = 0;
    double velocity;

    double ppr = 537.6;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        waitForStart();
        timer.reset();
        while(opModeIsActive()){
            position = motor.getCurrentPosition();
            position = position / (ppr / 360.0);

            time = timer.time();
            deltaT = time - lastTime;

            velocity = (position - lastPosition) / deltaT;
            dOutput = kD * velocity;

            lastPosition = position;
            lastTime = time;

            error = target - position;
            telemetry.addData("Error: ", error);

            errorSum = errorSum + (error * deltaT);

            pOutput = kP * (error/target);
            telemetry.addData("P Output: ", pOutput);

            iOutput = kI * (errorSum);

            power = pOutput + iOutput - dOutput;


            if(power > 0.5){
                power = 0.5;
            } else if (power < -0.5){
                power = -0.5;
            }

            telemetry.update();
            motor.setPower(power);
        }
        motor.setPower(0.0);
    }
}

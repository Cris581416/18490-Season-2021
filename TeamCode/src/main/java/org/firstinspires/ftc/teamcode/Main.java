package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.utilclasses.PIDController;

@TeleOp
public class Main extends LinearOpMode {
    DcMotor shooterMotor;
    DcMotor intakeMotor;
    double shooterPower = 0.6;
    double intakePower = 1.0;

    DcMotor tLMotor;
    DcMotor tRMotor;
    DcMotor bLMotor;
    DcMotor bRMotor;

    Drivetrain drivetrain;

    Servo pusher;
    final double SERVO_MIN = 0.9;
    double SERVO_MAX = 1.1;
    final long SERVO_DELAY = 425;

    final long B_DELAY = 500;

    PIDController shooterPID;
    double kP = 0.2;
    double kI = 0.0;
    double kD = 0.0;
    double target = 500.0;
    double PPR = 28.0;
    double MAX_RPM = 6000;
    double MAX_TPS = (MAX_RPM * PPR) / 60;

    ElapsedTime timer;

    double time_diff = 0;
    double tick_diff = 0;
    double prev_time = 0;
    double time = 0;
    double prev_ticks = 0;
    double ticks = 0;
    double speed = 0;

    boolean revving = true;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        tLMotor = hardwareMap.get(DcMotor.class, "tL");
        tRMotor = hardwareMap.get(DcMotor.class, "tR");
        bLMotor = hardwareMap.get(DcMotor.class, "bL");
        bRMotor = hardwareMap.get(DcMotor.class, "bR");

        drivetrain = new Drivetrain(tLMotor, tRMotor, bLMotor, bRMotor);

        pusher = hardwareMap.get(Servo.class, "pusher");

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        shooterPID = new PIDController(kP, kI, kD);

        shooterPID.setOutputRange(-1.0, 1.0);
        shooterPID.setInputRange(-MAX_TPS, MAX_TPS);
        shooterPID.setSetpoint(toTPS(target));

        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        // shooterPID.enable();
        while(opModeIsActive()){
            // Set current time and position (encoder ticks)
            time = timer.time();
            ticks = shooterMotor.getCurrentPosition();

            // Set differences between current time/ticks and previous time/ticks
            time_diff = time - prev_time;
            tick_diff = ticks - prev_ticks;

            // Get current rpm in ticks / seconds
            speed = (tick_diff) / (time_diff);

            // Increase or decrease intake speed through DPAD_RIGHT and DPAD_LEFT
            if(gamepad1.dpad_right && intakePower < 1.0){
                intakePower += 0.1;
                sleep(B_DELAY);
            } else if(gamepad1.dpad_left && intakePower > 0.0){
                intakePower -= 0.1;
                sleep(B_DELAY);
            }

            // Increase or decrease SERVO_MAX through RIGHT_BUMPER and LEFT_BUMPER
            if(gamepad1.right_bumper && SERVO_MAX < 2.0){
                SERVO_MAX += 0.1;
                sleep(B_DELAY);
            } else if(gamepad1.left_bumper && SERVO_MAX > 0.0){
                if(SERVO_MAX < 0.1){
                    SERVO_MAX = 0.0;
                } else{
                    SERVO_MAX -= 0.1;
                }
                sleep(B_DELAY);
            }

            // Intake
            if(gamepad1.a){
                intakeMotor.setPower(intakePower);
            } else {
                intakeMotor.setPower(0.0);
            }

            // Test SERVO_MAX movement
            if(gamepad1.x){
                pusher.setPosition(SERVO_MAX);
                sleep(SERVO_DELAY);
                pusher.setPosition(SERVO_MIN);
            }


            // PID Controller Section
            if(gamepad1.b){
                if(revving){
                    sleep(SERVO_DELAY * 3);
                    revving = false;
                } else{
                    shooterPower = shooterPID.performPID(speed);
                    shooterMotor.setPower(shooterPower);
                }
            } else{
                revving = true;
            }


            drivetrain.straferDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);


            //SERVO_MAX = (gamepad1.left_stick_x / 2.0) + 0.5;
            //pusher.setPosition(SERVO_MAX);
            // Add data to Driverstation for tuning
            telemetry.addData("Left Y: ", gamepad1.left_stick_y);
            telemetry.addData("Left X: ", gamepad1.left_stick_x);
            telemetry.addData("Right X: ", gamepad1.right_stick_x);
            telemetry.addData("Shooter Power: ", shooterPower);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("Servo Power: ", SERVO_MAX);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Error: ", shooterPID.getError());
            telemetry.update();

            // Set up the values for next loop's speed calculation
            prev_time = time;
            prev_ticks = ticks;
        }
        drivetrain.stop();
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        shooterPID.disable();
    }

    public double toTPS(double rpm){
        return (rpm * PPR) / 60;
    }
}

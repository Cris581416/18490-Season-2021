package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilclasses.Drivetrain;

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
    final double SERVO_MIN = 0;
    double SERVO_MAX = 0.2;
    final long SERVO_DELAY = 425;

    final long B_DELAY = 500;

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

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up && shooterPower < 1.0){
                shooterPower += 0.01;
                sleep(B_DELAY);
            } else if(gamepad1.dpad_down && shooterPower > 0.0){
                shooterPower -= 0.01;
                sleep(B_DELAY);
            }

            if(gamepad1.dpad_right && intakePower < 1.0){
                intakePower += 0.1;
                sleep(B_DELAY);
            } else if(gamepad1.dpad_left && intakePower > 0.0){
                intakePower -= 0.1;
                sleep(B_DELAY);
            }

            if(gamepad1.right_bumper && SERVO_MAX < 1.0){
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

            if(gamepad1.a){
                intakeMotor.setPower(intakePower);
            } else {
                intakeMotor.setPower(0.0);
            }

            /*if(gamepad1.b){
                telemetry.addLine("Shooting sequence started!");
                telemetry.update();
                shooterMotor.setPower(shooterPower);
                sleep(SERVO_DELAY);
                telemetry.addLine("Servo should be moving!");
                telemetry.update();
                pusher.setPosition(SERVO_MAX);
                sleep(SERVO_DELAY);
                pusher.setPosition(SERVO_MIN);
                sleep(SERVO_DELAY);
            } else{
                shooterMotor.setPower(0.0);
            }*/


            int shotCount = 0;
            while(gamepad1.b){
                telemetry.addLine("Shooting sequence started!");
                telemetry.update();
                shooterMotor.setPower(shooterPower);
                sleep(3 * SERVO_DELAY);
                while(gamepad1.b){
                    shooterMotor.setPower(shooterPower - (double)(shotCount * 0.002));
                    telemetry.addLine("Servo should be moving!");
                    telemetry.update();
                    pusher.setPosition(SERVO_MAX);
                    sleep(SERVO_DELAY);
                    pusher.setPosition(SERVO_MIN);
                    sleep(SERVO_DELAY);
                    shotCount += 1;
                }
            }

            if(gamepad1.x){
                pusher.setPosition(SERVO_MAX);
                sleep(SERVO_DELAY);
                pusher.setPosition(SERVO_MIN);
            }

            //drivetrain.straferDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //pusher.setPosition(SERVO_MAX);
            shooterMotor.setPower(0.0);

            telemetry.addData("Left Y: ", gamepad1.left_stick_y);
            telemetry.addData("Left X: ", gamepad1.left_stick_x);
            telemetry.addData("Right X: ", gamepad1.right_stick_x);
            telemetry.addData("Shooter Power: ", shooterPower);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("Servo Power: ", SERVO_MAX);
            telemetry.update();
        }
        drivetrain.stop();
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
    }
}

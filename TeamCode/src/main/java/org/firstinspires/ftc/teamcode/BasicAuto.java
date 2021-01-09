package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class BasicAuto extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    Servo armServo;

    double DRIVE_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        armServo = hardwareMap.get(Servo.class, "armServo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        driveForwardTimed(DRIVE_POWER, 4000);

        turnLeftTimed(DRIVE_POWER, 1000);

        driveForwardTimed(DRIVE_POWER, 4000);

        turnRightTimed(DRIVE_POWER, 1000);

        driveForwardTimed(DRIVE_POWER, 4000);

        armServo.setPosition(0.75);

        stopDriving();
    }

    public void driveForward(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void turnLeft(double power){
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
    }

    public void turnRight(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public void stopDriving(){
        driveForward(0.0);
    }

    public void driveForwardTimed(double power, long period){
        driveForward(power);
        sleep(period);
    }

    public void turnLeftTimed(double power, long period){
        turnLeft(power);
        sleep(period);
    }

    public void turnRightTimed(double power, long period){
        turnRight(power);
        sleep(period);
    }
}

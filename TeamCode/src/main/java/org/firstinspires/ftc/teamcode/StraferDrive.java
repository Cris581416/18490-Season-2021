package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class StraferDrive extends LinearOpMode {
    double tL;
    double tR;
    double bL;
    double bR;

    DcMotor tLMotor;
    DcMotor tRMotor;
    DcMotor bLMotor;
    DcMotor bRMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        tLMotor = hardwareMap.get(DcMotor.class, "tL");
        tRMotor = hardwareMap.get(DcMotor.class, "tR");
        bLMotor = hardwareMap.get(DcMotor.class, "bL");
        bRMotor = hardwareMap.get(DcMotor.class, "bR");

        tRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            tL = r * Math.cos(robotAngle) + rightX;
            tR = r * Math.sin(robotAngle) - rightX;
            bL = r * Math.sin(robotAngle) + rightX;
            bR = r * Math.cos(robotAngle) - rightX;

            tLMotor.setPower(tL);
            tRMotor.setPower(tR);
            bLMotor.setPower(bL);
            bRMotor.setPower(bR);

            telemetry.addData("tL: ", tL);
            telemetry.addData("tR: ", tR);
            telemetry.addData("bL: ", bL);
            telemetry.addData("bR: ", bR);
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Shooter extends LinearOpMode {
    DcMotor shooterMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            shooterMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Power: ", gamepad1.left_stick_y);
            telemetry.update();
        }

        shooterMotor.setPower(0.0);
    }
}

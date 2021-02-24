package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class SimpleStrafeDrive extends LinearOpMode {

    private DcMotor rear_l_motor;
    private DcMotor front_l_motor;
    private DcMotor rear_r_motor;
    private DcMotor front_r_motor;

    @Override
    public void runOpMode() {
        double Drive;
        double Turn;
        double Strafe;

        rear_l_motor = hardwareMap.dcMotor.get("bL");
        front_l_motor = hardwareMap.dcMotor.get("tL");
        rear_r_motor = hardwareMap.dcMotor.get("bR");
        front_r_motor = hardwareMap.dcMotor.get("tR");

        front_r_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Drive = -gamepad1.left_stick_y * 1;
                Turn = gamepad1.right_stick_x * 1;
                Strafe = gamepad1.left_stick_x * 1;

                rear_r_motor.setPower((Drive - Turn) - Strafe);
                rear_l_motor.setPower(Drive + Turn + Strafe);

                front_r_motor.setPower((Drive - Turn) + Strafe);
                front_l_motor.setPower((Drive + Turn) - Strafe);

                telemetry.addData("Left Pow", rear_r_motor.getPower());
                telemetry.addData("Right Pow", rear_l_motor.getPower());
                telemetry.update();
            }
        }
    }
}

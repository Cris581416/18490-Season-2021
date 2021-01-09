package org.firstinspires.ftc.teamcode.utilclasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {

    DcMotor tLMotor;
    DcMotor tRMotor;
    DcMotor bLMotor;
    DcMotor bRMotor;

    public Drivetrain(DcMotor tL, DcMotor tR, DcMotor bL, DcMotor bR){
        tLMotor = tL;
        tRMotor = tR;
        bLMotor = bL;
        bRMotor = bR;
    }

    public void straferDrive(double xspeed, double yspeed, double turnSpeed){
        double r = Math.hypot(-xspeed, yspeed);
        double robotAngle = Math.atan2(yspeed, -xspeed) - Math.PI / 4;
        double rightX = -turnSpeed;

        double tLSpeed = (r * Math.cos(robotAngle) + rightX);
        double tRSpeed = (r * Math.sin(robotAngle) - rightX);
        double bLSpeed = (r * Math.sin(robotAngle) + rightX);
        double bRSpeed = (r * Math.cos(robotAngle) - rightX);

        tLMotor.setPower(tLSpeed);
        tRMotor.setPower(tRSpeed);
        bLMotor.setPower(bLSpeed);
        bRMotor.setPower(bRSpeed);
    }

    public void arcadeDrive(double throttle, double turn){
        double leftSpeed = turn + throttle;
        double rightSpeed = turn - throttle;

        tLMotor.setPower(leftSpeed);
        tRMotor.setPower(rightSpeed);
        bLMotor.setPower(leftSpeed);
        bRMotor.setPower(rightSpeed);
    }
}

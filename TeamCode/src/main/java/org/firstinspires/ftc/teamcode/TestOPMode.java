package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class TestOPMode extends LinearOpMode {
    private Gyroscope imu;
    private DigitalChannel touchSensor;
    private RevColorSensorV3 colorSensor;
    private Servo servoTest;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double tgtPower = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.y) {
                servoTest.setPosition(0);
            } else if(gamepad1.a) {
                servoTest.setPosition(0.8);
            } else if(gamepad1.x || gamepad1.b){
                servoTest.setPosition(0.5);
            }
            telemetry.addData("Servo Position: ", servoTest.getPosition());
            telemetry.addData("Distance (cm): ", colorSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Color Sensed: ", colorSensor.getLightDetected());
            if(touchSensor.getState() == false){
                telemetry.addData("Touch Sensor", "Pressed");
                servoTest.setPosition(0.5);
            } else{
                telemetry.addData("Touch Sensor: ", "Not Pressed");
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

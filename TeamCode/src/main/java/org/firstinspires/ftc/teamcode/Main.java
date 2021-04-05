package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utilclasses.Drivetrain;
import org.firstinspires.ftc.teamcode.utilclasses.GroupTelemetry;
import org.firstinspires.ftc.teamcode.utilclasses.PIDController;
import org.firstinspires.ftc.teamcode.utilclasses.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.concurrent.TimeUnit;

@TeleOp
@Config
public class Main extends LinearOpMode {
    DcMotorEx shooterMotor;
    DcMotor intakeMotor;
    double shooterPower = 0.6;
    double intakePower = 1.0;

    DcMotor tLMotor;
    DcMotor tRMotor;
    DcMotor bLMotor;
    DcMotor bRMotor;

    Drivetrain drivetrain;

    VoltageSensor batteryVoltageSensor;

    Servo pusher;
    double SERVO_MIN = 0.075;
    double SERVO_MAX = 0.5;
    double SERVO_INC = 0.025;
    final long SERVO_DELAY = 175;
    final long B_DELAY = 500;

    PIDFCoefficients coefficients = new PIDFCoefficients(80, 0.35, 2.4, 12.5);
    double target = 3100.0;
    double PIDCounter = 0;

    PIDController aligner = new PIDController(0.9, 0.0, 0.0);
    public static double dkP = 0.65;

    ElapsedTime timer;

    boolean revving = true;

    static OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        //intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        tLMotor = hardwareMap.get(DcMotor.class, "tL");
        tRMotor = hardwareMap.get(DcMotor.class, "tR");
        bLMotor = hardwareMap.get(DcMotor.class, "bL");
        bRMotor = hardwareMap.get(DcMotor.class, "bR");

        drivetrain = new Drivetrain(tLMotor, tRMotor, bLMotor, bRMotor);

        pusher = hardwareMap.get(Servo.class, "pusher");

        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        shooterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType motorConfigurationType = shooterMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        shooterMotor.setMotorType(motorConfigurationType);

        shooterMotor.setVelocityPIDFCoefficients(coefficients.p, coefficients.i, coefficients.d, coefficients.f);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        Utils.setPIDFCoefficients(shooterMotor, coefficients, batteryVoltageSensor);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        final GoalDetector processor = new GoalDetector();
        webcam.setPipeline(processor);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(processor.FRAME_WIDTH * 2, 360, OpenCvCameraRotation.UPRIGHT);
            }
        });


        timer.reset();
        while(opModeIsActive()){
            // Increase or decrease intake speed through DPAD_RIGHT and DPAD_LEFT
            if(gamepad1.dpad_right && intakePower < 1.0){
                intakePower += 0.1;
                sleep(B_DELAY);
            } else if(gamepad1.dpad_left && intakePower > 0.0){
                intakePower -= 0.1;
                sleep(B_DELAY);
            }

            // Increase or decrease SERVO_MAX through RIGHT_BUMPER and LEFT_BUMPER
            if(gamepad1.right_bumper && SERVO_MIN < SERVO_MAX){
                SERVO_MIN += SERVO_INC;
                sleep(B_DELAY);
            } else if(gamepad1.left_bumper && SERVO_MIN > 0.0){
                if(SERVO_MAX < SERVO_INC){
                    SERVO_MIN = 0.0;
                } else{
                    SERVO_MIN -= SERVO_INC;
                }
                sleep(B_DELAY);
            }

            // Intake
            if(gamepad1.left_trigger != 0.0){
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

            // PID Shooter Controller Sequence
            if(gamepad1.right_trigger != 0.0){
                if(revving){
                    disableMotors();
                    shooterMotor.setVelocity(Utils.rpm_to_tps(target));
                    sleep(SERVO_DELAY * 3);
                    revving = false;
                } else{
                    shooterMotor.setVelocity(Utils.rpm_to_tps(target));
                    pusher.setPosition(SERVO_MAX);
                    sleep(SERVO_DELAY);
                    pusher.setPosition(SERVO_MIN);
                    sleep(2 * SERVO_DELAY);
                }
            } else{
                revving = true;
                shooterMotor.setMotorDisable();
            }

            if(gamepad1.b){
                double turn = aligner.calculate(processor.getTx(), timer.time());

                telemetry.addData("PID Output: ", turn);
                drivetrain.arcadeDrive(0.0, turn);
            } else{
                drivetrain.straferDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }

            // Add data to driver station for tuning
            telemetry.addData("Left Y: ", gamepad1.left_stick_y);
            telemetry.addData("Left X: ", gamepad1.left_stick_x);
            telemetry.addData("Right X: ", -gamepad1.right_stick_x);
            telemetry.addData("Revving: ", revving);
            telemetry.addData("Intake Power: ", intakePower);
            telemetry.addData("Servo Power: ", SERVO_MIN);
            telemetry.addData("Tx: ", processor.getTx());
            TelemetryPacket packet = new TelemetryPacket();
            if (processor.goalBox != null){
                packet.put("Box Width", processor.goalBox.width);
                packet.put("Box Height", processor.goalBox.height);
                packet.put("Box Area", processor.goalBox.height * processor.goalBox.width);
                //packet.put("Perspective", processor.getPerspective());
                //packet.put("Distance", processor.getTd());
                packet.put("Angle", processor.getAngle());
                telemetry.addData("Box Width: ", processor.goalBox.width);
                telemetry.addData("Box Height: ", processor.goalBox.height);
            }
            packet.put("Velocity", shooterMotor.getVelocity());
            packet.put("Target", Utils.rpm_to_tps(target));
            packet.put("Tx: ", processor.getTx());
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        drivetrain.stop();
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
    }

    public void disableMotors(){
        shooterMotor.setMotorDisable();
        intakeMotor.setPower(0.0);
        drivetrain.stop();
    }





    /**
     * This is the pipeline class used to detect the ring goals
     * and get their center point.
     *
     */
    public static class GoalDetector extends OpenCvPipeline
    {
        Size matSize = new Size();
        boolean viewportPaused;

        int widener = 10;
        int lowH = 0; //95;
        int highH = 35 + widener; //134;
        int lowS = 90 -  widener; //72;
        int highS = 245 + widener; //213;
        int lowV = 40 - widener; //67;
        int highV = 255;

        final int FRAME_HEIGHT = 240;
        final int FRAME_WIDTH = 320;

        private final int RATIO = 3;
        private final int KERNEL_SIZE = 3;
        private final Size BLUR_SIZE = new Size(4,4);

        // Contour Thresholds
        private int lowThresh = 0;
        private int centerSize = 4;
        private double minArea = 135 * 85;
        private double minRatio = 1.75;
        double minWidth = 80;
        double minHeight = 80;


        private double desiredWidth = 143.0; // Pixels
        private double desiredHeight = 82.0; // Pixels

        Random rng = new Random(12345);

        // Random Mats
        Mat hierarchy = new Mat();
        Mat hsvImg = new Mat();
        Mat thresholdedImg = new Mat();
        Mat sideBySideImg = new Mat();
        Mat drawing = new Mat();
        Mat dst = new Mat();
        Mat resizedDst = new Mat();

        // Center Point
        Point center;

        // Object Bounding Boxes
        Rect goalBox;
        RotatedRect minGoalBox;

        int startGC = 50;
        int output;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */
            output = 0;

            Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);
            Core.inRange(hsvImg, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), thresholdedImg);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(thresholdedImg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            RotatedRect[] minRect = new RotatedRect[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                minRect[i] = Imgproc.minAreaRect(new MatOfPoint2f(contoursPoly[i].toArray()));
            }

            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }

            drawing = Mat.zeros(hsvImg.size(), CvType.CV_8UC3);

            boolean setNewCenter = false;
            for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                Rect rect = boundRect[i];
                double width = rect.br().x - rect.tl().x;
                double height = rect.br().y - rect.tl().y;

                if(rect.area() > minArea && (width / height) < minRatio) {
                    Imgproc.drawContours(drawing, contoursPolyList, i, color);
                    Imgproc.rectangle(drawing, rect.tl(), rect.br(), color, 2);
                    center = new Point((rect.tl().x + rect.br().x) / 2, (rect.tl().y + rect.br().y) / 2);
                    Imgproc.rectangle(drawing, new Point(center.x - centerSize, center.y - centerSize),
                            new Point(center.x + centerSize, center.y + centerSize), new Scalar(255, 255, 255), Imgproc.FILLED);
                    goalBox = rect;
                    minGoalBox = minRect[i];
                    setNewCenter = true;
                }
            }

            if(!setNewCenter){
                center = null;
            }

            List<Mat> src = Arrays.asList(hsvImg, drawing);
            Core.hconcat(src, dst);
            dst.push_back(Mat.zeros(new Size(dst.width(), 480), CvType.CV_8UC3));
            Imgproc.resize(dst, resizedDst, new Size(1280, 720), 0);
            sideBySideImg = resizedDst.clone();

            System.gc();
            System.runFinalization();

            return sideBySideImg;
        }

        public double centerX(){
            if(center != null){
                return center.x;
            } else return FRAME_WIDTH / 2;
        }

        public double getTx(){
            return centerX() - (double)(FRAME_WIDTH / 2);
        }

        public int getFrameHeight(){
            return sideBySideImg.height();
        }

        public int getFrameWidth(){
            return sideBySideImg.width();
        }

        public double getTd(){
            return 1.0 - (goalBox.height * desiredWidth) / (goalBox.width * desiredHeight);
        }

        public double getAngle(){

            double flatRatio = desiredWidth / desiredHeight;
            double currentRatio = (double) goalBox.width / (double) goalBox.height;

            double angle = (1 - (currentRatio / flatRatio)) * 90;
            angle *= getPerspective();

            return minGoalBox.angle;
        }

        private int getPerspective(){

            Mat imgSection = new Mat(thresholdedImg, goalBox);

            int per = 0;

            int rows = imgSection.rows();
            int cols = imgSection.cols();

            for(int row = 0; row < rows; row++){
                for(int col = 0; col < cols; col++){
                    if(imgSection.get(row, col)[0] == 255){
                        if(row < row / 2){
                            per += 1;
                        } else{
                            per -= 1;
                        }
                    }
                }
            }

            if(per != 0){
                return (per / Math.abs(per));
            }

            return 0;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}

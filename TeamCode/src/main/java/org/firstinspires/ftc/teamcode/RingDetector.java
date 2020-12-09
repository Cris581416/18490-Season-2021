package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import java.math.*;

@TeleOp
public class RingDetector extends LinearOpMode
{
    final int FRAME_HEIGHT = 240;
    final int FRAME_WIDTH = 320;

    OpenCvCamera webcam;
    //Servo servo;

    DcMotor tLMotor;
    DcMotor tRMotor;
    DcMotor bLMotor;
    DcMotor bRMotor;

    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        //servo = hardwareMap.get(Servo.class, "servo1");
        tLMotor = hardwareMap.get(DcMotor.class, "tL");
        tRMotor = hardwareMap.get(DcMotor.class, "tR");
        bLMotor = hardwareMap.get(DcMotor.class, "bL");
        bRMotor = hardwareMap.get(DcMotor.class, "bR");

        //tRMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline processor = new SamplePipeline();
        webcam.setPipeline(processor);

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            /* telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps()); */
            telemetry.addData("Tx: ", processor.getTx());
            telemetry.addData("Low H: ", processor.lowH);
            telemetry.addData("High H: ", processor.highH);

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            if(gamepad1.b){
                if(processor.highH > 0){
                    processor.highH -= 5;
                }
            }

            if(gamepad1.x){
                if(processor.lowH < 180){
                    processor.lowH += 5;
                }
            }

            if(gamepad1.left_bumper){
                if(processor.lowH > 0){
                    processor.lowH -= 5;
                }
            }

            if(gamepad1.right_bumper){
                if(processor.highH < 180){
                    processor.highH += 5;
                }
            }

            // Set servo
            //servo.setPosition(0.5 + (processor.getTx()) / (FRAME_WIDTH / 2));
            double tLSpeed;
            double bLSpeed;
            double tRSpeed;
            double bRSpeed;

            if(gamepad1.y) {
                double driveSpeed = (processor.getTx()) / (FRAME_WIDTH / 2) / 2;
                if (processor.getTx() == 0) {
                    driveSpeed = 0;
                }
                telemetry.addData("Tx Drive Speed: ", driveSpeed);

                tLSpeed = -driveSpeed - -gamepad1.left_stick_y;
                bLSpeed = -driveSpeed - -gamepad1.left_stick_y;

                tRSpeed = driveSpeed - -gamepad1.left_stick_y;
                bRSpeed = -driveSpeed - gamepad1.left_stick_y;
            } else{
                double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
                double rightX = -gamepad1.right_stick_x;

                tLSpeed = (r * Math.cos(robotAngle) + rightX);
                tRSpeed = (r * Math.sin(robotAngle) - rightX);
                bLSpeed = (r * Math.sin(robotAngle) + rightX);
                bRSpeed = (r * Math.cos(robotAngle) - rightX);

                tLMotor.setPower(tLSpeed);
                tRMotor.setPower(tRSpeed);
                bLMotor.setPower(bLSpeed);
                bRMotor.setPower(bRSpeed);
            }

            telemetry.addData("tlSpeed: ", tLSpeed);
            telemetry.addData("bLSpeed", bLSpeed);
            telemetry.addData("tRSpeed", tRSpeed);
            telemetry.addData("bRSpeed", bRSpeed);

            tLMotor.setPower(tLSpeed);
            bLMotor.setPower(bLSpeed);

            tRMotor.setPower(tRSpeed);
            bRMotor.setPower(bRSpeed);

            telemetry.update();
            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        Size matSize = new Size();
        boolean viewportPaused;
        int lowH = 100;
        int highH = 160;
        int lowS = 0;
        int highS = 255;
        int lowV = 0;
        int highV = 255;
        private int RATIO = 3;
        private int KERNEL_SIZE = 3;
        private Size BLUR_SIZE = new Size(4,4);
        private int lowThresh = 0;
        private int centerSize = 4;
        double minWidth = 80;
        double minHeight = 80;
        Random rng = new Random(12345);

        // Random Mats
        Mat hierarchy = new Mat();
        Mat hsvImg = new Mat();
        Mat thresholdedImg = new Mat();

        // Center Point
        Point center;

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

            //Mat hsvImg = new Mat();
            //Mat blurredImg = new Mat();
            //Mat thresholdedImg = new Mat();
            //Mat detectedEdges = new Mat();
            //Mat processedImg;

            Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);
            //Imgproc.blur(hsvImg, blurredImg, BLUR_SIZE);
            Core.inRange(hsvImg, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV), thresholdedImg);
            //Imgproc.Canny(thresholdedImg, detectedEdges, lowThresh, lowThresh * RATIO, KERNEL_SIZE, false);
            //processedImg = new Mat(hsvImg.size(), CvType.CV_8UC3, Scalar.all(0));
            //thresholdedImg.copyTo(processedImg, detectedEdges);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(thresholdedImg, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            Mat drawing = Mat.zeros(new Size(FRAME_WIDTH, FRAME_HEIGHT), CvType.CV_8UC3);
            for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
                if(boundRect[i].width >= minWidth && boundRect[i].height >= minHeight) {
                    Imgproc.drawContours(drawing, contoursPolyList, i, color);
                    Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
                    center = new Point((boundRect[i].tl().x + boundRect[i].br().x) / 2, (boundRect[i].tl().y + boundRect[i].br().y) / 2);
                    Imgproc.rectangle(drawing, new Point(center.x - centerSize, center.y - centerSize),
                            new Point(center.x + centerSize, center.y + centerSize), new Scalar(255, 255, 255), Imgproc.FILLED);
                    output += 1;
                }
            }

            //hsvImg.release();
            //blurredImg.release();
            //processedImg.release();
            //thresholdedImg.release();
            //detectedEdges.release();
            //hierarchy.release();

            contours = null;
            contoursPoly = null;
            contoursPolyList = null;

            startGC --;
            if (startGC == 0) {
                System.gc();
                System.runFinalization();
                startGC = 50;
            }

            return drawing;
        }

        public double centerX(){
            if(center != null){
                return center.x;
            } else return FRAME_WIDTH / 2;
        }

        public double getTx(){
            return centerX() - (double)(FRAME_WIDTH / 2);
        }

        public double getHeight(){
            return thresholdedImg.size().height;
        }

        public double getWidth(){
            return thresholdedImg.size().width;
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

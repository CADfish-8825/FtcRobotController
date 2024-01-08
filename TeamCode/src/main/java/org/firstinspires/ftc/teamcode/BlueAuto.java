package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class BlueAuto extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    double cX = -1; // Use -1 to indicate no detection initially
    public DcMotor FL;
    public DcMotor BL;
    public DcMotor FR;
    public DcMotor BR;
    public DcMotor pitch;
    public DcMotor slide;
    public Servo claw;
    public Servo tilt;


    HardwareMapPage robot = new HardwareMapPage();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    double cY = 0;
    double width = 0;

    public static final double objectWidthInRealWorldUnits = 3.75;
    public static final double focalLength = 728;

    double leftThreshold=500;
    double rightThreshold=1000;



    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        robot.init(hardwareMap);

        FL = hardwareMap.get(DcMotor.class, "leftfront");
        BL = hardwareMap.get(DcMotor.class, "leftback");
        FR = hardwareMap.get(DcMotor.class, "rightfront");
        BR = hardwareMap.get(DcMotor.class, "rightback");
        pitch = hardwareMap.get(DcMotor.class, "pitchMotor");
        slide = hardwareMap.get(DcMotor.class, "slideMotor");
        claw = hardwareMap.get(Servo.class,"Claw");
        tilt = hardwareMap.get(Servo.class,"tilt");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new BlueCubePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
        while (!isStarted() && !isStopRequested()) {
            if (opModeIsActive()) {
                telemetry.addData("has vision", cX);
                driveCamera(cX);
                telemetry.update();

            }
        }
    }
//}

    public void driveCamera(double cX) {
        boolean noCube = false;
        if (cX < leftThreshold) {
            telemetry.addLine("driving left");
            driveLeft();
        } else if (cX > rightThreshold || noCube == true) {
            telemetry.addLine("driving right");
            driveRight();
        } else if (cX < rightThreshold && cX > leftThreshold) {
            telemetry.addLine("driving center");
            driveCenter();
        }
    }


    public void driveRight() {
        CLOSECLAW();
        move(.8, -0.5, -0.5);
        move(0.6, -0.5, 0.5);
        move(0.5, -0.5, -0.5);
        sleep(300);
        move(0.55, 0.5, 0.5);
        sleep(300);
        move( 1.4, 0.5,-0.5);
        SCORETILT();
        usePitch(-1,800);
        move(1.9, -0.5, -0.5);
        sleep(300);
        move(0.6,0.25,-0.25);
        sleep(100);
        move(0.6,-0.5,-0.5);
        OPENCLAW();
        sleep(300);
        move(0.4,0.3,0.3);
        move(1.4,-0.3,0.3);
        move(0.7,-0.5,-0.5);

    }

    public void driveLeft() {
        CLOSECLAW();
        move(0.7, -0.5, -0.5);
        move(0.6,0.5,-0.5);
        move(0.5,-0.5,-0.5);
        sleep(300);
        move(0.4, 0.5, 0.5);
        move(0.65, 0.5, -0.5);
        SCORETILT();
        usePitch(-1,800);
        move(2.3,-0.5,-0.5);
        sleep(400);
        OPENCLAW();
        sleep(500);
        move(.3,0.5,0.5);
        move(1.4,0.5,-0.5);
        sleep(200);
        move(0.5,-0.5,-0.5);


    }

    public void driveCenter() {
      CLOSECLAW();
      move(1.7, -0.4, -0.4);
      sleep(500);
      move(0.3,0.5,0.5);
      move(1.25,0.5,-0.5);
      SCORETILT();
      usePitch(-1,800);
      move(2.4,-0.3,-0.3);
      OPENCLAW();
      move(1.3,-0.5,0.5);
      move(1,-0.5,-0.5);
    }


    public void move(double distance, double left_power, double right_power) {
        //532 is equivalent to 1ft
        distance = distance * 532;
        int edistance = Math.toIntExact(Math.round(distance));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (left_power > 0) {
            FL.setTargetPosition(-edistance);
            BL.setTargetPosition(-edistance);
        } else {
            FL.setTargetPosition(edistance);
            BL.setTargetPosition(edistance);
        }
        if (right_power > 0) {
            FR.setTargetPosition(-edistance);
            BR.setTargetPosition(-edistance);
        } else {
            FR.setTargetPosition(edistance);
            BR.setTargetPosition(edistance);
        }
        FL.setPower(left_power);
        BL.setPower(left_power);
        FR.setPower(right_power);
        BR.setPower(right_power);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.isBusy()) {
            telemetry.addData("busy", distance);
            telemetry.update();
        }
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        sleep(150);
    }

    private void strafe(double distance, double fL, double fR, double bL, double bR) {
        //532 is equivalent to 1ft
        distance = distance * 532;
        int edistance = Math.toIntExact(Math.round(distance));
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (fL > 0) {
            FL.setTargetPosition(edistance);
        } else {
            FL.setTargetPosition(-edistance);
        }
        if (fR > 0) {
            FR.setTargetPosition(edistance);
        } else {
            FR.setTargetPosition(-edistance);
        }
        if (bL > 0) {
            BL.setTargetPosition(edistance);
        } else {
            BL.setTargetPosition(-edistance);
        }
        if (bR > 0) {
            BL.setTargetPosition(edistance);
        } else {
            BL.setTargetPosition(-edistance);
        }
        FL.setPower(fL);
        FR.setPower(fR);
        BL.setPower(bL);
        BR.setPower(bR);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (FL.isBusy()) {
            telemetry.addData("busy", distance);
            telemetry.update();
        }
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        sleep(250);
    }

    private void usePitch (double pow, double dist) {
        int encdist = Math.toIntExact(Math.round(dist));
        pitch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (pow > 0) {
            pitch.setTargetPosition(encdist);
        } else {
            pitch.setTargetPosition(-encdist);
        }
        pitch.setPower(pow);
        pitch.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (pitch.isBusy()) {
            telemetry.addData("busy", dist);
            telemetry.update();
        }
        pitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch.setPower(0);
    }
    private void useSlide (double pow, double dist) {
        int encdist = Math.toIntExact(Math.round(dist));
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (pow > 0) {
            slide.setTargetPosition(encdist);
        } else {
            slide.setTargetPosition(-encdist);
        }
        slide.setPower(pow);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slide.isBusy()) {
            telemetry.addData("busy", dist);
            telemetry.update();
        }
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setPower(0);
    }

    private void CLOSECLAW () {
        claw.setPosition(1);
        sleep(500);
    }
    private void OPENCLAW () {
        claw.setPosition(0.7);
        sleep(500);
    }
    private void SCORETILT () {
        tilt.setPosition(1);
    }

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees) {
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 1) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.setMotorPower(0, 0, 0, 0);
    }

    public void turnTo(double degrees) {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
        robot.setAllPower(0);
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.0055, 0.00036, 0.014);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 ||  pid.getLastSlope() > 0.75
        ) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }

    class BlueCubePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat blueMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a blue outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
                if (cX < leftThreshold) {
                    telemetry.addLine("left");
                    telemetry.update();

                }else if (cX < rightThreshold && cX > leftThreshold) {
                    telemetry.addLine("middle");
                    telemetry.update();
                } else if (cX > rightThreshold) {
                    telemetry.addLine("right");
                    telemetry.update();
                }
                telemetry.update();
            }



            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerBlue = new Scalar(10, 50, 50);
            Scalar upperBlue = new Scalar(80, 255, 255);


            Mat blueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}
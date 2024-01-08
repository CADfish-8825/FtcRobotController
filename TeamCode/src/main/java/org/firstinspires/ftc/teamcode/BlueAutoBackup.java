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
public class BlueAutoBackup extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    double lastDetectedcubeX = -1; // Use -1 to indicate no detection initially
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
                telemetry.addData("has vision", lastDetectedcubeX);
                driveCamera(lastDetectedcubeX);
                telemetry.update();

            }
        }
    }
//}

    public void driveCamera(double cubeX) {
        double noCube = 0;
        double leftThreshold = 500;
        double rightThreshold = 1000;
        if (cubeX < leftThreshold) {
            telemetry.addData("driving","left");
            driveLeft();
        } else if (cubeX > rightThreshold || cubeX < noCube) {
            telemetry.addLine("driving right");
            driveRight();
        } else if (cubeX < rightThreshold && cubeX > leftThreshold) {
            telemetry.addLine("driving center");
            driveCenter();
        }
    }


    public void driveRight() {
        CLOSECLAW();
        move(1.05, -0.5, -0.5);
        move(0.7, -0.5, 0.5);
        move(0.5, -0.5, -0.5);
        sleep(300);
        move(0.55, 0.5, 0.5);
        sleep(300);
        move( 1.5, 0.5,-0.5);
        SCORETILT();
        usePitch(-1,500);
        move(1.6, -0.5, -0.5);
        sleep(300);
        move(0.5,0.25,-0.25);
        sleep(100);
        move(0.7,-0.5,-0.5);
        OPENCLAW();
    }

    public void driveLeft() {
        CLOSECLAW();
        move(0.7, -0.5, -0.5);
        move(0.4,0.5,-0.5);
        move(0.5,-0.5,-0.5);
        sleep(300);
        move(0.55, 0.5, 0.5);
        move(0.75, 0.5, -0.5);
        SCORETILT();
        usePitch(-1,800);
        move(2.3,-0.5,-0.5);
        sleep(400);
        OPENCLAW();
        sleep(500);
        move(.3,0.5,0.5);
        move(1.6,0.5,-0.5);
        sleep(200);
        move(0.3,-0.5,-0.5);


    }

    public void driveCenter() {
      CLOSECLAW();
      move(1.7, -0.4, -0.4);
      sleep(500);
      move(0.3,0.5,0.5);
      move(1.25,0.5,-0.5);
      SCORETILT();
      usePitch(-1,500);
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
            // Define lower and upper bounds for cyan/light blue in HSV color space
            Scalar lowercyan = new Scalar(80, 50, 50); // Adjust these values
            Scalar uppercyan = new Scalar(100, 255, 255); // Adjust these values

            // Convert the input image to the HSV color space
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);

            // Create a binary mask to identify cyan/light blue pixels
            Mat cyanMask = new Mat();
            Core.inRange(input, lowercyan, uppercyan, cyanMask);

            // Apply morphological operations to reduce noise
            Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(cyanMask, cyanMask, Imgproc.MORPH_CLOSE, morphKernel);
            Imgproc.morphologyEx(cyanMask, cyanMask, Imgproc.MORPH_OPEN, morphKernel);

            // Find contours in the binary mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(cyanMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Filter the contours to select cube-like shapes
            List<MatOfPoint> cubeContours = new ArrayList<>();
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > 1000) { // Adjust this threshold based on your cube size
                    cubeContours.add(contour);
                }
            }

            // Process detected cube contours
            for (MatOfPoint cubeContour : cubeContours) {
                // Calculate the center point of the cyan cube
                Moments moments = Imgproc.moments(cubeContour);
                double cubeX = moments.get_m10() / moments.get_m00();
                double centerY = moments.get_m01() / moments.get_m00();

                // Display telemetry for the cyan cube's location
                telemetry.addData("Cyan Cube X", cubeX);
                telemetry.addData("Cyan Cube Y", centerY);
                telemetry.update();

                lastDetectedcubeX = cubeX;

                // Draw rectangles around the detected cubes on the original image
                Rect rect = Imgproc.boundingRect(cubeContour);
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(255, 255, 0), 2); // Cyan color
            }

            return input;
        }
    }


}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop")
    public class teleop extends LinearOpMode {

        public DcMotor FL;
        public DcMotor BL;
        public DcMotor FR;
        public DcMotor BR;
        public DcMotor pitch;
        public DcMotor slide;
        public Servo claw;
        public Servo tilt;
        public Servo launch;

        final float normalPower = 0.7f;
        final float lowerPower = 0.4f;

        @Override
        public void runOpMode() {
            float turn_FL_X = 0;
            float turn_BR_X = 0;
            float turn_FR_X = 0;
            float turn_BL_X = 0;
            float strafe_FR_X = 0;
            float strafe_BL_X = 0;
            float strafe_BR_X = 0;
            float strafe_FL_X = 0;
            float strafe_FL_Y = 0;
            float strafe_FR_Y = 0;
            float strafe_BL_Y = 0;
            float strafe_BR_Y = 0;
            double driveSpeed = 1.0;

            FL = hardwareMap.get(DcMotor.class, "leftfront");
            BL = hardwareMap.get(DcMotor.class, "leftback");
            FR = hardwareMap.get(DcMotor.class, "rightfront");
            BR = hardwareMap.get(DcMotor.class, "rightback");
            pitch = hardwareMap.get(DcMotor.class, "pitchMotor");
            slide = hardwareMap.get(DcMotor.class, "slideMotor");
            tilt = hardwareMap.get(Servo.class, "tilt");
            claw = hardwareMap.get(Servo.class,"Claw");
            launch = hardwareMap.get(Servo.class, "launch");


            waitForStart();
            if (opModeIsActive()) {
                FL.setDirection(DcMotorSimple.Direction.FORWARD);
                BL.setDirection(DcMotorSimple.Direction.FORWARD);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);//
                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                while (opModeIsActive()) {
                    telemetry.addData("leftstickX", gamepad1.left_stick_x);
                    telemetry.addData("leftstickY", gamepad1.left_stick_y);
                    telemetry.addData("rightstickX", gamepad1.right_stick_x);
                    telemetry.update();
                    //lower power for more precise robot movement
                    if (gamepad1.x) {
                        if (gamepad1.left_stick_y > 0.1) {
                            // forward
                            strafe_BR_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_FL_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_FR_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_BL_Y = gamepad1.left_stick_y * lowerPower;
                        } else if (gamepad1.left_stick_y < -0.1) {
                            // backward
                            strafe_BR_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_FL_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_FR_Y = gamepad1.left_stick_y * lowerPower;
                            strafe_BL_Y = gamepad1.left_stick_y * lowerPower;
                        } else if (gamepad1.left_stick_x > 0.1) {
                            // left turn
                            turn_FL_X = -gamepad1.left_stick_x * lowerPower;
                            turn_FR_X = gamepad1.left_stick_x * lowerPower;
                            turn_BL_X = -gamepad1.left_stick_x * lowerPower;
                            turn_BR_X = gamepad1.left_stick_x * lowerPower;
                        } else if (gamepad1.left_stick_x < -0.1) {
                            // right turn
                            turn_FL_X = -gamepad1.left_stick_x * lowerPower;
                            turn_FR_X = gamepad1.left_stick_x * lowerPower;
                            turn_BL_X = -gamepad1.left_stick_x * lowerPower;
                            turn_BR_X = gamepad1.left_stick_x * lowerPower;
                        } else {
                            turn_FL_X = 0;
                            turn_FR_X = 0;
                            turn_BL_X = 0;
                            turn_BR_X = 0;
                            strafe_FL_Y = 0;
                            strafe_FR_Y = 0;
                            strafe_BL_Y = 0;
                            strafe_BR_Y = 0;

                        }
                    } else {
                        if (gamepad1.left_stick_y > 0.1) {
                            // forward
                            strafe_BR_Y = gamepad1.left_stick_y * normalPower;
                            strafe_FL_Y = gamepad1.left_stick_y * normalPower;
                            strafe_FR_Y = gamepad1.left_stick_y * normalPower;
                            strafe_BL_Y = gamepad1.left_stick_y * normalPower;
                        } else if (gamepad1.left_stick_y < -0.1) {
                            // backward
                            strafe_BR_Y = gamepad1.left_stick_y * normalPower;
                            strafe_FL_Y = gamepad1.left_stick_y * normalPower;
                            strafe_FR_Y = gamepad1.left_stick_y * normalPower;
                            strafe_BL_Y = gamepad1.left_stick_y * normalPower;
                        } else if (gamepad1.left_stick_x > 0.1) {
                            // left turn
                            turn_FL_X = -gamepad1.left_stick_x * normalPower;
                            turn_FR_X = gamepad1.left_stick_x * normalPower;
                            turn_BL_X = -gamepad1.left_stick_x * normalPower;
                            turn_BR_X = gamepad1.left_stick_x * normalPower;
                        } else if (gamepad1.left_stick_x < -0.1) {
                            // right turn
                            turn_FL_X = -gamepad1.left_stick_x * normalPower;
                            turn_FR_X = gamepad1.left_stick_x * normalPower;
                            turn_BL_X = -gamepad1.left_stick_x * normalPower;
                            turn_BR_X = gamepad1.left_stick_x * normalPower;
                        } else {
                            turn_FL_X = 0;
                            turn_FR_X = 0;
                            turn_BL_X = 0;
                            turn_BR_X = 0;
                            strafe_FL_Y = 0;
                            strafe_FR_Y = 0;
                            strafe_BL_Y = 0;
                            strafe_BR_Y = 0;

                        }
                    }
                    // turn
                    if (gamepad1.right_stick_y > 0.1) {
                        // right strafe
                        strafe_FL_X = gamepad1.right_stick_y;
                        strafe_FR_X = -gamepad1.right_stick_y;
                        strafe_BL_X = -gamepad1.right_stick_y;
                        strafe_BR_X = gamepad1.right_stick_y;
                    } else if (gamepad1.right_stick_y < -0.1) {
                        // left strafe
                        strafe_FL_X = gamepad1.right_stick_y;
                        strafe_FR_X = -gamepad1.right_stick_y;
                        strafe_BL_X = -gamepad1.right_stick_y;
                        strafe_BR_X = gamepad1.right_stick_y;

                    } else {
                        strafe_FL_X = 0;
                        strafe_FR_X = 0;
                        strafe_BL_X = 0;
                        strafe_BR_X = 0;
                    }

                    // pitch
                    if (gamepad1.left_bumper) {
                        pitch.setPower(1);
                    }
                    else if (gamepad1.right_bumper) {
                        pitch.setPower(-.5);
                    }
                    else {
                        pitch.setPower(0);
                    }
                    // slide
                    if (gamepad1.dpad_up) {
                        slide.setPower(-1);
                    }
                    else if (gamepad1.dpad_down) {
                        slide.setPower(1);
                    }
                    else if (gamepad1.b) {
                        slide.setPower(-0.3);
                    }else {
                        slide.setPower(0);
                    }
                    // claw
                    if (gamepad1.a) {
                        claw.setPosition(0.6);
                    }
                    else {
                        claw.setPosition(1);
                    }
                    // tilt
                    if (gamepad1.dpad_left) {
                        tilt.setPosition(0.8);
                    }
                    else if (gamepad1.dpad_right) {
                        tilt.setPosition(9);
                    }
                    //launch drone
                    if (gamepad1.y) {
                        launch.setPosition(200);
                    }
                    //set engine to power in hanging
//                    if (gamepad1.right_trigger) {
//
//                    }

                    FL.setPower(driveSpeed * (turn_FL_X + strafe_FL_X + strafe_FL_Y));
                    FR.setPower(driveSpeed * (turn_FR_X + strafe_FR_X + strafe_FR_Y));
                    BL.setPower(driveSpeed * (turn_BL_X + strafe_BL_X + strafe_BL_Y));
                    BR.setPower(driveSpeed * (turn_BR_X + strafe_BR_X + strafe_BR_Y));
                }
            }
        }
    }

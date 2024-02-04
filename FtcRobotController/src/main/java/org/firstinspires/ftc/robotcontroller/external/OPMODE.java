/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OPMODE", group="Linear OpMode")
//Disabled
public class OPMODE extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private DcMotor intake = null;


    private Servo airplane = null;
    private Servo leftGateLifter = null;
    private Servo rightGateLifter = null;
    private Servo magazineSpinner = null;
    private Servo magazineRelease = null;
    private Servo intakeServo = null;
    private Servo pixelPusher = null;

    public final static double OP_C = 2;

    public final static double ARM_MIN = -1.0;
    public final static double ARM_MAX = 1.0;
    final double ARM_SPEED = 0.001;

    public float leftLinearSlidePos;
    public float rightLinearSlidePos;

    private double tmp = 0;
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        tmp = x;
        x = y;
        y = yaw;
        yaw = tmp;
        double leftFrontPower    = y + x + yaw;
        double rightFrontPower   = y - x - yaw;
        double leftBackPower     = y - x + yaw;
        double rightBackPower    = y + x - yaw;
        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    @Override
    public void runOpMode() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, "intake_spin");

        airplane = hardwareMap.get(Servo.class, "airplane_release");
        leftGateLifter = hardwareMap.get(Servo.class, "left_gate_lifter");
        rightGateLifter = hardwareMap.get(Servo.class, "right_gate_lifter");
        magazineSpinner = hardwareMap.get(Servo.class, "spin");
        magazineRelease = hardwareMap.get(Servo.class, "release");
        pixelPusher = hardwareMap.get(Servo.class, "pusher");


        double armPosition = 0.5;
        double lifterPosition = 0.5;
        double drive;
        double strafe;
        double turn;

        boolean intakeDown = false;
        boolean intakeOn = false;
        boolean stationaryMag = true;

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // linear slides
            float linearPower = gamepad2.left_stick_y/2;
            leftLinearSlidePos = leftLinearSlide.getCurrentPosition();
            rightLinearSlidePos = rightLinearSlide.getCurrentPosition();

            if (-460 < leftLinearSlidePos && linearPower < 0) {
                leftLinearSlide.setPower((linearPower));
                rightLinearSlide.setPower((linearPower));

                if (-200.0f < leftLinearSlidePos) {
                    magazineSpinner.setPosition(0.656);
                    stationaryMag = false;
                }
                else if (-350.0f < leftLinearSlidePos) {
                    magazineSpinner.setPosition(0.038);
                    stationaryMag = false;
                }
                else if (leftLinearSlidePos > -460) {
                    magazineSpinner.setPosition(0.231);
                    stationaryMag = false;
                }
                else {
                    stationaryMag = true;
                }


            }
            else if (linearPower > 0) {
                leftLinearSlide.setPower((linearPower));
                rightLinearSlide.setPower((linearPower));
                if (-30 > leftLinearSlidePos) {
                    magazineSpinner.setPosition(0.656);
                    stationaryMag = false;
                }
                else {
                    stationaryMag = true;
                }
            }
            else {
                leftLinearSlide.setPower(0.0);
                rightLinearSlide.setPower(0.0);
            }

            // pixel pusher mechanism
            if (gamepad2.a) {
                pixelPusher.setPosition(0.75);
            }
            else {
                pixelPusher.setPosition(0.4);
            }


            // magazine

            if (stationaryMag) {
                magazineSpinner.setPosition(0.741);
            }


            // upside down: magazineSpinner.setPosition(0.038);
            // drop angle: magazineSpinner.setPosition(0.231);

            if (gamepad2.left_bumper) {
                magazineRelease.setPosition(0.35);
            }
            else {
                magazineRelease.setPosition(0.717);
            }


            // intake activation
            if (intakeDown) {
                intakeServo.setPosition(0.664);
            }
            else {
                intakeServo.setPosition(0.062);
            }
            if (gamepad2.x) {
                intakeDown = true;
            }
            if (gamepad2.b) {
                intakeOn = !intakeOn;
            }
            if (intakeOn) {
                intake.setPower(0.5);
            }
            else {
                intake.setPower(0.0);
            }

            // airplane launcher
            if (gamepad2.right_bumper) {
                airplane.setPosition(0.540);
            }
            else {
                airplane.setPosition(0.610);
            }

            // gate lifter
            if (gamepad2.right_trigger >= 0.5) {
                lifterPosition += ARM_SPEED;
            }
            else if (gamepad2.left_trigger >= 0.5) {
                lifterPosition -= ARM_SPEED;
            }
            lifterPosition = Range.clip(lifterPosition, ARM_MIN, ARM_MAX);
            rightGateLifter.setPosition(lifterPosition);
            leftGateLifter.setPosition(1-lifterPosition);


            if (gamepad1.a) {
                armPosition += ARM_SPEED;
            }
            else if (gamepad1.y) {
                armPosition -= ARM_SPEED;
            }
            armPosition = Range.clip(armPosition, ARM_MIN, ARM_MAX);


            drive  = -gamepad1.left_stick_y  / OP_C;
            strafe = -gamepad1.left_stick_x  / OP_C;
            turn   = -gamepad1.right_stick_x / OP_C;
            moveRobot(drive, strafe, turn);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Magazine Position", magazineSpinner.getPosition());
            telemetry.addData("Linear Slide Positions", leftLinearSlidePos);
            telemetry.addData("Linear Slide Power", linearPower);
            telemetry.addData("Servo Position", armPosition);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }}

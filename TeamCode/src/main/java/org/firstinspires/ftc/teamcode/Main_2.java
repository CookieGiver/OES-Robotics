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

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Main", group="Linear OpMode")
//Disabled
public class Main_2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
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

    public final static double ARM_MIN = -1.0;
    public final static double ARM_MAX = 1.0;
    final double ARM_SPEED = 0.001;

    public float leftLinearSlidePos;
    public float rightLinearSlidePos;


     
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLinearSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightLinearSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");

        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "intake_spin");

        airplane = hardwareMap.get(Servo.class, "airplane_release");
        leftGateLifter = hardwareMap.get(Servo.class, "left_gate_lifter");
        rightGateLifter = hardwareMap.get(Servo.class, "right_gate_lifter");
        magazineSpinner = hardwareMap.get(Servo.class, "spin");
        magazineRelease = hardwareMap.get(Servo.class, "release");
        pixelPusher = hardwareMap.get(Servo.class, "pusher");


        double armPosition = 0.5;
        double lifterPosition = 0.5;

        boolean intakeDown = false;
        boolean intakeOn = false;
        boolean stationaryMag = true;


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD );
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


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // linear slides
            double linearPower = gamepad2.left_stick_y/2;
            float leftlinearSlidePos = leftLinearSlide.getCurrentPosition();
            float rightLinearSlidePos = rightLinearSlide.getCurrentPosition();

            if (-565.0 < leftLinearSlidePos && linearPower < 0) {
                leftLinearSlide.setPower((linearPower));
                rightLinearSlide.setPower((linearPower));

                if (-200 < leftLinearSlidePos) {
                    magazineSpinner.setPosition(0.656);
                    stationaryMag = false;
                }
                else if (-400 < leftLinearSlidePos) {
                    magazineSpinner.setPosition(0.038);
                    stationaryMag = false;
                }
                else if (leftLinearSlidePos > -580) {
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
            //magazineRelease.setPosition(armPosition);

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Magazine Position", magazineSpinner.getPosition());
            telemetry.addData("Linear Slide Positions", leftlinearSlidePos);
            telemetry.addData("Linear Slide Power", linearPower);
            telemetry.addData("Servo Position", armPosition);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}

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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 * <p>
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at
 * <a href="https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html">GM0</a>
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 * <p>
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 * <p>
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 * <p>
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 * <p>
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "DisasterDrive", group = "Robot")
public class DisasterDrive extends LinearOpMode {
    private static final double LEFT_STICK_Y_DEAD_ZONE = 0.1;
    private static final double LEFT_STICK_X_DEAD_ZONE = 0.1;
    private static final double RIGHT_STICK_X_DEAD_ZONE = 0.1;
    private static final double RIGHT_STICK_Y_DEAD_ZONE = 0.1;
    private static final double SLOW_FACTOR = .5; // halve the speed
    private static final int ARM_MOTOR_LIMIT = 5000;
    private static final int SLIDE_MOTOR_LIMIT = 13900;

    // Adjust power for a defined dead zone
    private double adjustPower(double power, double deadZone) {
        double adjustedPower = 0;
        if (Math.abs(power) > deadZone) {
            adjustedPower = (power - Math.signum(power) * deadZone) / (1 - deadZone);
        }
        return adjustedPower;
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "br");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotor winchMotor = hardwareMap.get(DcMotor.class, "winch"); // motor 2
        winchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor hangMotor = hardwareMap.get(DcMotor.class, "hang"); // motor 3
        hangMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        CRServo collectServo = hardwareMap.get(CRServo.class, "collect");

        IMU imu = hardwareMap.get(IMU.class, "imu");
//        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for driver to press Play
        waitForStart();

        boolean yButtonDown = false;
        boolean slowMode = false;

        // Run until driver presses Stop
        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x; // turn
            double leftStickY = -gamepad1.left_stick_y; // arm
            double rightStickX = gamepad1.right_stick_x; // strafe
            double rightStickY = -gamepad1.right_stick_y; // slide
            double rightTrigger = gamepad1.right_trigger; // forward
            double leftTrigger = gamepad1.left_trigger; // backward (both to brake)
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean dpad_up = gamepad1.dpad_up;
            boolean dpad_down = gamepad1.dpad_down;
            boolean dpad_right = gamepad1.dpad_right;
            boolean dpad_left = gamepad1.dpad_left;
            boolean aButton = gamepad1.a; // collection in
            boolean bButton = gamepad1.b; // collection out
            boolean yButton = gamepad1.y; // slow mode
            boolean xButton = gamepad1.x;

            // slow mode on & off
            if (yButton) {
                yButtonDown = true;
            }
            if (!yButton && yButtonDown) {
                yButtonDown = false;
                slowMode = !slowMode;
            }

            // collection motor intake/outtake
            if (aButton) {
                collectServo.setPower(-1.0);
            } else if (bButton) {
                collectServo.setPower(1.0);
            } else {
                collectServo.setPower(0.0);
            }

            // arm motor raise/lower
            double armMotorPower = adjustPower(leftStickY, LEFT_STICK_Y_DEAD_ZONE);
            if (armMotorPower > 0) {
                armMotor.setTargetPosition(ARM_MOTOR_LIMIT);
            } else {
                armMotor.setTargetPosition(0);
            }
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(armMotorPower);

            // slide motor out/in
            double slideMotorPower = adjustPower(rightStickY, LEFT_STICK_Y_DEAD_ZONE);
            if (slideMotorPower > 0) {
                slideMotor.setTargetPosition(SLIDE_MOTOR_LIMIT);
            } else {
                slideMotor.setTargetPosition(0);
            }
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setPower(slideMotorPower);

            //Winch Control
            if (dpad_up) {
                winchMotor.setPower(100.0);
                telemetry.addData("Winch Up", "");
            }
            if (dpad_down) {
                winchMotor.setPower(-100.0);
                telemetry.addData("Winch down", "");
            }

            //Hang Control
            if (dpad_right) {
                hangMotor.setPower(100.0);
                telemetry.addData("Hang Out", "");
            }
            if (dpad_left) {
                hangMotor.setPower(-100.0);
                telemetry.addData("Hang In", "");
            }

            // driving control
            double forwardPower = rightTrigger - leftTrigger;
            if (rightTrigger > 0 && leftTrigger > 0) {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (forwardPower < 0) {
                    forwardPower = 0;
                }
            } else {
                frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            double turnPower = -adjustPower(leftStickX, LEFT_STICK_X_DEAD_ZONE);
            double strafePower = adjustPower(rightStickX, RIGHT_STICK_X_DEAD_ZONE);
            double denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
            if (slowMode) {
                denominator /= SLOW_FACTOR;
            }
            double frontLeftPower = (forwardPower + strafePower - turnPower) / denominator;
            double frontRightPower = (forwardPower - strafePower + turnPower) / denominator;
            double backLeftPower = (forwardPower - strafePower - turnPower) / denominator;
            double backRightPower = (forwardPower + strafePower + turnPower) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("Arm Power", "%.2f", armMotorPower);
            telemetry.addData("Arm Encoder", "%d", armMotor.getCurrentPosition());
            telemetry.addData("Slide Power", "%.2f", slideMotorPower);
            telemetry.addData("Slide Encoder", "%d", slideMotor.getCurrentPosition());
            telemetry.addData("Slow Mode: ", "%b", slowMode);
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.update();
        }
    }
}

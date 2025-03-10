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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    private static final int ARM_MOTOR_LIMIT = 3200; // 3161
    private static final int SLIDE_MOTOR_LIMIT = 1900;


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

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(6);
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        Servo light = hardwareMap.get(Servo.class, "light");

        double maxAllowedInches = 41.5;
        double armHeightInches = 13.5;
        double ticksAtMaxAllowedLength = 1200;
        double encoderAtHorizontal = 678;
        double encoderAtVertical = 3181;
        boolean isArmReset = false;

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
            double leftTrigger = gamepad1.left_trigger; // backward (both to brake)
            double rightTrigger = gamepad1.right_trigger; // forward
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean dpad_up = gamepad1.dpad_up;
            boolean dpad_down = gamepad1.dpad_down;
            boolean dpad_left = gamepad1.dpad_left;
            boolean dpad_right = gamepad1.dpad_right;
            boolean aButton = gamepad1.a; // collection in
            boolean bButton = gamepad1.b; // collection out
            boolean xButton = gamepad1.x;
            boolean yButton = gamepad1.y; // slow mode

            double slowPlayerTwo = 0.333;

            leftStickX = Math.min(leftStickX + gamepad2.left_stick_x * slowPlayerTwo, 1);
            leftStickY = Math.min(leftStickY + -gamepad2.left_stick_y * slowPlayerTwo, 1);
            rightStickX = Math.min(rightStickX + gamepad2.right_stick_x * slowPlayerTwo, 1);

            rightStickY = Math.min(rightStickY + -gamepad2.right_stick_y, 1);
            leftTrigger = Math.min(leftTrigger + gamepad2.left_trigger, 1);
            rightTrigger = Math.min(rightTrigger + gamepad2.right_trigger, 1);
            leftBumper = leftBumper || gamepad2.left_bumper;
            rightBumper = rightBumper || gamepad2.right_bumper;
            dpad_up = dpad_up || gamepad2.dpad_up;
            dpad_down = dpad_down || gamepad2.dpad_down;
            dpad_left = dpad_left || gamepad2.dpad_left;
            dpad_right = dpad_right || gamepad2.dpad_right;
            aButton = aButton || gamepad2.a;
            bButton = bButton || gamepad2.b;
            xButton = xButton || gamepad2.x;
            yButton = yButton || gamepad2.y;

            if (xButton)
            {
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
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
                collectServo.setPower(1.0);
            } else if (bButton) {
                collectServo.setPower(-1.0);
            } else {
                collectServo.setPower(0.0);
            }

            // arm motor raise/lower
            double armMotorPower = adjustPower(rightStickY, LEFT_STICK_Y_DEAD_ZONE);
            if (armMotorPower > 0) {
                armMotor.setTargetPosition(ARM_MOTOR_LIMIT);
            } else {
                armMotor.setTargetPosition(-10000);
            }

            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setPower(armMotorPower);
            // slide motor out/in
            double slideMotorPower = adjustPower(rightStickY, LEFT_STICK_Y_DEAD_ZONE);

            double ninetyDegreeRange = encoderAtVertical - encoderAtHorizontal;
            double angleDegrees = ((armMotor.getCurrentPosition() - encoderAtHorizontal) / ninetyDegreeRange) * 90;
            double raiseLimit = maxAllowedInches / Math.cos(angleDegrees * Math.PI / 180);

            int limit = Math.min(SLIDE_MOTOR_LIMIT, (int) (raiseLimit * (ticksAtMaxAllowedLength / maxAllowedInches)));
//            if (angleDegrees < 0) {
//                double loweredLimit = armHeightInches/Math.sin((90+angleDegrees) * Math.PI / 180);
//                limit = Math.min(limit, (int) (loweredLimit * (ticksAtMaxAllowedLength / maxAllowedInches)));
//            }
            if (rightTrigger == 0 && leftTrigger == 0) {
                slideMotor.setPower(0);
            }

            if (slideMotor.getCurrentPosition() > limit) {
                slideMotor.setTargetPosition(limit);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(-1.0);
            }
            if (angleDegrees > 90) {
                slideMotor.setTargetPosition(-10000);
                slideMotor.setPower(-1.0);

            }

            if (rightTrigger > 0) {
                slideMotor.setTargetPosition(limit);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(rightTrigger);
            }
            if (leftTrigger > 0) {
                slideMotor.setTargetPosition(-10000);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(-leftTrigger);
            }

            // winch control
            if (!dpad_up && !dpad_down) {
                winchMotor.setPower(0);
            }
            if (dpad_up) {
                winchMotor.setTargetPosition(5000);
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchMotor.setPower(1.0);
                telemetry.addData("Winch Up", "");
            }
            if (dpad_down) {
                winchMotor.setTargetPosition(0);
                winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchMotor.setPower(-1.0);
                telemetry.addData("Winch down", "");
            }

            // hang control
            if (!dpad_right && !dpad_left) {
                hangMotor.setPower(0);
            }
            if (dpad_right) {
                hangMotor.setTargetPosition(10000);
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor.setPower(1.0);
                telemetry.addData("Hang Out", "");
            }
            if (dpad_left) {
                hangMotor.setTargetPosition(-10300);
                hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hangMotor.setPower(-1.0);
                telemetry.addData("Hang In", "");
            }

            // driving control
            double forwardPower = leftStickY;
            double turnPower = -adjustPower(rightStickX, RIGHT_STICK_X_DEAD_ZONE);
            double strafePower = -adjustPower(leftStickX, LEFT_STICK_X_DEAD_ZONE);
            if (rightBumper) {
                strafePower = 0.3;
            } else if (leftBumper) {
                strafePower = -0.3;
            }

            double denominator = Math.max(Math.abs(forwardPower) + Math.abs(strafePower) + Math.abs(turnPower), 1);
            if (slowMode) {
                denominator /= SLOW_FACTOR;
            }
            double frontLeftPower = (forwardPower - strafePower - turnPower) / denominator;
            double frontRightPower = (forwardPower + strafePower + turnPower) / denominator;
            double backLeftPower = (forwardPower + strafePower - turnPower) / denominator;
            double backRightPower = (forwardPower - strafePower + turnPower) / denominator;
            // hard brake!

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Get the normalized colors from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            final float[] hsvValues = new float[3];
            Color.colorToHSV(colors.toColor(), hsvValues);

            /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
             * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
             * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
             * for an explanation of HSV color. */

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine();

            if (colors.red > 0.01 && colors.red > colors.blue * 2 && colors.red > colors.green * 2) {
                telemetry.addData("Red", "");
                light.setPosition(.28);
            } else if (colors.red > 0.015 && colors.green > 0.015 && colors.blue < colors.red) {
                telemetry.addData("Yellow", "");
                light.setPosition(.37);
            } else if (colors.blue > 0.015 && colors.blue > colors.red && colors.blue > colors.green) {
                telemetry.addData("Blue","");
                light.setPosition(.611);
            } else {
                light.setPosition(0.0);
            }
//            light.setPosition((hsvValues[0] / 180) + .279);

            // send the info back to driver station using telemetry function.
            if (touchSensor.isPressed() && !isArmReset) {
                telemetry.addData("Touch Sensor", "Is Pressed");
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setPower(0);
                isArmReset = true;
//            } else {
////                telemetry.addData("Touch Sensor", "Is Not Pressed");
            }


            telemetry.addData("Arm Power", "%.2f", armMotorPower);
            telemetry.addData("Arm Encoder", "%d", armMotor.getCurrentPosition());
            telemetry.addData("Slide Power", "%.2f", slideMotorPower);
            telemetry.addData("Slide Encoder", "%d", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Limit", "%d", limit);
            telemetry.addData("Angle Degrees","%.2f", angleDegrees );
            telemetry.addData("Slow Mode: ", "%b", slowMode);
            Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);
            telemetry.addData("Angular Velocity", "%.1f", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.update();
        }
    }
}

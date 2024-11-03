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
import com.qualcomm.robotcore.hardware.Servo;

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

@TeleOp(name = "DriveTest", group = "Robot")
public class DriveTest extends LinearOpMode {
    private boolean flipDirection = false;
    private boolean yButtonDown = false;

    @Override
    public void runOpMode() {
        // Initialize Motors
        /* Declare OpMode members. */
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "br");
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");

        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CRServo collectServo = hardwareMap.get(CRServo.class, "collect");

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for driver to press Play
        waitForStart();
        slideMotor.setTargetPosition(10);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);

        // Run until driver presses Stop
        while (opModeIsActive()) {

            double leftStickY = gamepad1.left_stick_y;
            double leftStickX = -gamepad1.left_stick_x;
            double rightStickX = -gamepad1.right_stick_x;
            double rightStickY = -gamepad1.right_stick_y;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger = gamepad1.left_trigger;
            boolean rightBumper = gamepad1.right_bumper;
            boolean leftBumper = gamepad1.left_bumper;
            boolean dpad_up = gamepad1.dpad_up;
            boolean dpad_down = gamepad1.dpad_down;
            boolean yButton = gamepad1.y;
            boolean xButton = gamepad1.x;
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;

            if (yButton) {
                yButtonDown = true;
            }
            if (!yButton && yButtonDown) {
                yButtonDown = false;
                flipDirection = !flipDirection;
            }

            // run collection motor intake
            if (aButton) {
                collectServo.setPower(-1.0);
            } else if (bButton) {
                collectServo.setPower(1.0);
            } else {
                collectServo.setPower(0.0);
            }

            if (dpad_up) {
                armMotor.setPower(-0.5);
            } else {
                armMotor.setPower(0.0);
            }
            if (dpad_down) {
                armMotor.setPower(0.5);
            } else {
                armMotor.setPower(0.0);
            }

            // stop motors if both triggers are not in use
            // !leftBumper && !rightBumper &&
//            if (leftTrigger == 0 && rightTrigger == 0) {
//                scissorMotor.setPower(0);
//                slideMotor.setPower(0);
//            }
//
//            // go up and out if right trigger is pressed
//            if (rightTrigger > 0) {
//                scissorMotor.setTargetPosition(-5000);
//                scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                scissorMotor.setPower(1);
//
//                slideMotor.setTargetPosition(1450);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(rightTrigger);
//            }

            // go back and down if left trigger is pressed
//            if (leftTrigger > 0) {
//                slideMotor.setTargetPosition(10);
//                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideMotor.setPower(leftTrigger);
//
//                scissorMotor.setTargetPosition(0);
//                scissorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                scissorMotor.setPower(1);
//            }

            // Strafe right (or left if direction is flipped)
            if (rightBumper) {
                leftStickX = 0.5 * (flipDirection ? 1 : -1);
            }
            // Strafe right (or right if direction is flipped)
            if (leftBumper) {
                leftStickX = 0.5 * (flipDirection ? -1 : 1);
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            if (flipDirection) {
                leftStickX *= -1;
                leftStickY *= -1;
            }

            // add a dead zone to the left joystick strafe
            if (Math.abs(leftStickX) < 0.2){
                leftStickX = 0;
            }

            double denominator = Math.max(Math.abs(leftStickY) + Math.abs(leftStickX) + Math.abs(rightStickX), 1);
            double frontLeftPower = (leftStickY + leftStickX + rightStickX) / denominator;
            double frontRightPower = (leftStickY - leftStickX - rightStickX) / denominator;
            double backLeftPower = (leftStickY - leftStickX + rightStickX) / denominator;
            double backRightPower = (leftStickY + leftStickX - rightStickX) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Send telemetry message to signify robot running;
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("FL encoder", "%d", frontLeftMotor.getCurrentPosition());

            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("FR encoder", "%d", frontRightMotor.getCurrentPosition());

            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("BL encoder", "%d", backLeftMotor.getCurrentPosition());

            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addData("BR encoder", "%d", backRightMotor.getCurrentPosition());

//            telemetry.addData("Scissor lift encoder", "%d", scissorMotor.getCurrentPosition());

            telemetry.addData("Slide encoder", "%d", slideMotor.getCurrentPosition());

            telemetry.addData("Flipped direction?", "%b", flipDirection);
            telemetry.update();
        }
    }
}

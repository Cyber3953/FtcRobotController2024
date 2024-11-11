package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Timer;

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

@TeleOp(name = "Reset", group = "Robot")
public class Reset extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting
        telemetry.addData(">", "Reset Ready.  Press Play.");    //
        telemetry.update();

        // Wait for driver to press Play
        waitForStart();

        double start = System.nanoTime();
        double armMotorPower = -0.1;
        double slideMotorPower = -0.1;

        // Run until driver presses Stop
        while (opModeIsActive()) {
            armMotor.setPower(armMotorPower);
            slideMotor.setPower(slideMotorPower);
            double armMotorVelocity = armMotor.getVelocity();
            double slideMotorVelocity = slideMotor.getVelocity();
            double armMotorCurrent = armMotor.getCurrent(CurrentUnit.MILLIAMPS);
            double slideMotorCurrent = slideMotor.getCurrent(CurrentUnit.MILLIAMPS);
            if (start + 50 < System.nanoTime()) {
                if (armMotorVelocity < 3 || armMotorCurrent > 500) {
                    armMotorPower = 0;
                }
                if (slideMotorVelocity < 3 || slideMotorCurrent > 500) {
                    slideMotorPower = 0;
                }
            }

            if (armMotorPower == 0 && slideMotorPower == 0) {
                stop();
            }

            telemetry.addData("Arm Power", "%.2f", armMotorPower);
            telemetry.addData("Arm Encoder", "%d", armMotor.getCurrentPosition());
            telemetry.addData("Arm Velocity", "%.2f", armMotorVelocity);
            telemetry.addData("Arm Current", "%.2f", armMotorCurrent);
            telemetry.addData("Slide Power", "%.2f", slideMotorPower);
            telemetry.addData("Slide Encoder", "%d", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Velocity", "%.2f", slideMotorVelocity);
            telemetry.addData("Slide Current", "%.2f", slideMotorCurrent);
            telemetry.update();
        }
    }
}

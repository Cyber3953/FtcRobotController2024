package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "HardwareTest", group = "Robot")
public class HardwareTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        int selector = 0;

        // Initialize motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "fl");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "fr");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "bl");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "br");
        DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "slide");

        CRServo collectServo = hardwareMap.get(CRServo.class, "collect");
//        Servo deliveryServo = hardwareMap.get(Servo.class, "Delivery");
//        Servo releaseServo = hardwareMap.get(Servo.class, "Gate");
//        Servo droneServo = hardwareMap.get(Servo.class, "Drone");
//
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
//
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean aButtonDown = false;
        boolean bButtonDown = false;

        while (opModeIsActive()) {
            double leftStickY = -gamepad1.left_stick_y;
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;

            if (aButton) {
                aButtonDown = true;
            }
            if (!aButton && aButtonDown) {
                aButtonDown = false;
                selector++;
            }
            if (bButton) {
                bButtonDown = true;
            }
            if (!bButton && bButtonDown) {
                bButtonDown = false;
                selector--;
                if (selector == -1) selector = 6;
            }
            switch (selector) {
                case 0:
                    frontLeftMotor.setPower(leftStickY);
                    telemetry.addData("Front left motor: ", "%.2f", leftStickY);
                    telemetry.addData("Front left encoder", "%d", frontLeftMotor.getCurrentPosition());
                    break;
                case 1:
                    backRightMotor.setPower(leftStickY);
                    telemetry.addData("Back right motor: ", "%.2f", leftStickY);
                    telemetry.addData("Back right encoder", "%d", backRightMotor.getCurrentPosition());
                    break;
                case 2:
                    frontRightMotor.setPower(leftStickY);
                    telemetry.addData("Front right motor: ", "%.2f", leftStickY);
                    telemetry.addData("Front right encoder", "%d", frontRightMotor.getCurrentPosition());
                    break;
                case 3:
                    backLeftMotor.setPower(leftStickY);
                    telemetry.addData("Back left motor: ", "%.2f", leftStickY);
                    telemetry.addData("Back left encoder", "%d", backLeftMotor.getCurrentPosition());
                    break;
                case 4:
                    armMotor.setPower(leftStickY);
                    telemetry.addData("Arm motor: ", "%.2f", leftStickY);
                    break;
                case 5:
                    slideMotor.setPower(leftStickY);
                    telemetry.addData("Slide motor: ", "%.2f", leftStickY);
                    break;
                case 6:
                    collectServo.setPower(leftStickY);
                    telemetry.addData("Collection servo: ", "%.2f", leftStickY);
                    break;
//                case 8:
//                    // this is not currently connected to the robot
//                    //droneServo.setPosition(leftStickY);
//                    telemetry.addData("Drone servo:", "%.2f", leftStickY);
//                    break;
//                case 9:
//                    deliveryServo.setPosition(leftStickY);
//                    telemetry.addData("Delivery servo: ", "%.2f", leftStickY);
//                    break;
//                case 10:
//                    releaseServo.setPosition(leftStickY);
//                    telemetry.addData("Release servo:", "%.2f", leftStickY);
//                    break;
//                case 11:
//                    NormalizedRGBA colors = colorSensor.getNormalizedColors();
//                    telemetry.addLine()
//                            .addData("Red", "%.3f", colors.red)
//                            .addData("Green", "%.3f", colors.green)
//                            .addData("Blue", "%.3f", colors.blue);
//                    break;
//                case 12:
//                    droneServo.setPosition(leftStickY);
//                    telemetry.addData("Drone servo:", "%.2f", leftStickY);
                default:
                    selector = 0;
                    break;
            }
            telemetry.update();
        }
    }
}

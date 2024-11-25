package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TestHardware", group = "Robot")
public class TestHardware extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        int selector = 0;

        // Initialize motors
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl"); // motor 2
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr"); // motor 3
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl"); // motor 0
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "br"); // motor 1
        DcMotorEx armMotorPrimary = hardwareMap.get(DcMotorEx.class, "arm"); // motor 0
//        DcMotorEx armMotorSecondary = hardwareMap.get(DcMotorEx.class, "arm2");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide"); // motor 1
        DcMotorEx winchMotor = hardwareMap.get(DcMotorEx.class, "winch"); // motor 2
        DcMotorEx hangMotor = hardwareMap.get(DcMotorEx.class, "hang"); // motor 3

        CRServo collectServo = hardwareMap.get(CRServo.class, "collect"); // servo 0
//        Servo deliveryServo = hardwareMap.get(Servo.class, "Delivery");
//
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
//
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotorPrimary.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorPrimary.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean aButtonDown = false;
        boolean bButtonDown = false;
        String selectedHardware = "";
        DcMotorEx selectedMotor = null;

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
                if (selector == -1) selector = 8;
            }
            switch (selector) {
                case 0:
                    selectedHardware = "Front Left Motor";
                    selectedMotor = frontLeftMotor;
                    break;
                case 1:
                    selectedHardware = "Back Right Motor";
                    selectedMotor = backRightMotor;
                    break;
                case 2:
                    selectedHardware = "Front Right Motor";
                    selectedMotor = frontRightMotor;
                    break;
                case 3:
                    selectedHardware = "Back Left Motor";
                    selectedMotor = backLeftMotor;
                    break;
                case 4:
                    selectedHardware = "Arm Worm Drive";
                    selectedMotor = armMotorPrimary;

                    armMotorPrimary.setPower(leftStickY);
//                    armMotorSecondary.setPower(leftStickY);

                    telemetry.addData(selectedHardware, "");
                    telemetry.addData("Power: ", "%.2f", selectedMotor.getPower());
                    telemetry.addData("Encoder: ", "%d", selectedMotor.getCurrentPosition());
                    telemetry.addData("Velocity: ", "%.2f", selectedMotor.getVelocity());
                    telemetry.addData("Current: ", "%.2f", selectedMotor.getCurrent(CurrentUnit.MILLIAMPS));
                    break;
                case 5:
                    selectedHardware = "Slide Motor";
                    selectedMotor = slideMotor;
                    break;
                case 6:
                    selectedHardware = "Collection Servo";
                    collectServo.setPower(leftStickY);
                    telemetry.addData("Collection servo: ", "%.2f", leftStickY);
                    break;
                case 7:
                    selectedHardware = "Winch Motor";
                    selectedMotor = winchMotor;
                    break;
                case 8:
                    selectedHardware = "Hang Motor";
                    selectedMotor = hangMotor;
                    break;
//                case 7:
//                    deliveryServo.setPosition(leftStickY);
//                    telemetry.addData("Delivery servo: ", "%.2f", leftStickY);
//                    break;
//                case 8:
//                    NormalizedRGBA colors = colorSensor.getNormalizedColors();
//                    telemetry.addLine()
//                            .addData("Red", "%.3f", colors.red)
//                            .addData("Green", "%.3f", colors.green)
//                            .addData("Blue", "%.3f", colors.blue);
//                    break;
                default:
                    selector = 0;
                    break;
            }
            if (selectedHardware.toLowerCase().contains("motor")) {
                selectedMotor.setPower(leftStickY);
                telemetry.addData(selectedHardware, "");
                telemetry.addData("Power: ", "%.2f", selectedMotor.getPower());
                telemetry.addData("Encoder: ", "%d", selectedMotor.getCurrentPosition());
                telemetry.addData("Velocity: ", "%.2f", selectedMotor.getVelocity());
                telemetry.addData("Current: ", "%.2f", selectedMotor.getCurrent(CurrentUnit.MILLIAMPS));
            }

            telemetry.update();
        }
    }
}

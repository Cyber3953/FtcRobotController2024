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
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        DcMotorEx slideMotor = hardwareMap.get(DcMotorEx.class, "slide");

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
                if (selector == -1) selector = 6;
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
                    selectedHardware = "Arm Motor";
                    selectedMotor = backLeftMotor;
                    break;
                case 5:
                    selectedHardware = "Slide Motor";
                    selectedMotor = slideMotor;
                    break;
                case 6:
                    collectServo.setPower(leftStickY);
                    telemetry.addData("Collection servo: ", "%.2f", leftStickY);
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

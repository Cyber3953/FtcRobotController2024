package org.firstinspires.ftc.teamcode.drive.red;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "left_side")
public class left_side extends LinearOpMode {

    DcMotorEx armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    Servo light = null;
    NormalizedColorSensor colorSensor;
    TouchSensor touchSensor;
    ElapsedTime time = new ElapsedTime();
    double servo_timeout = 0d;
    double current_time = 0d;
    double color = 0d;
    float MAX_SERVO_TIMEOUT = 1000f;
    float gain = 6;
    boolean activate_lift = true;
    Pose2d artemis = new Pose2d(49.8, 47.3, Math.toRadians(45));

    @SuppressLint("DefaultLocale")
    private boolean color_results()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        if (colors.red > 0.01 && colors.red > colors.blue * 2 && colors.red > colors.green * 2) {
            return true;
        } else if (colors.red > 0.015 && colors.green > 0.015 && colors.blue < colors.red) {
            return true;
        } else if (colors.blue > 0.015 && colors.blue > colors.red && colors.blue > colors.green) {
            return true;
        }
        return false;
    }

    private void rainbow(double increment)
    {
        color = light.getPosition();
        light.setPosition(color += increment);

        if (light.getPosition() > 0.722)
        {
            light.setPosition(0.280);
        }
    }

    @SuppressLint("DefaultLocale")
    private void boom()
    {
        current_time = System.currentTimeMillis();

        boolean result = color_results();

        do {
            collectServo.setPower(0.66);
            servo_timeout = System.currentTimeMillis() - current_time;
            ez_tel(String.format("servo is spinning t: %f", servo_timeout));
            rainbow(0.001);
        } while (servo_timeout < MAX_SERVO_TIMEOUT && (color_results() || !result));

        collectServo.setPower(0i);
    }

    @SuppressLint("DefaultLocale")
    private void reverse_boom()
    {
        current_time = System.currentTimeMillis();

        do {
            collectServo.setPower(-1);
            servo_timeout = System.currentTimeMillis() - current_time;
            ez_tel(String.format("servo is spinning t: %f", servo_timeout));
            rainbow(0.001);
        } while (servo_timeout < MAX_SERVO_TIMEOUT && !color_results());

        collectServo.setPower(0);
    }

    @SuppressLint("DefaultLocale")
    private void lift_the_lift(boolean slightly)
    {
        if (!activate_lift) { return; }

        if (slightly) {
            armMotor.setTargetPosition(2300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(300);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            armMotor.setTargetPosition(2735);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(1849);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        armMotor.setPower(1);
        slideMotor.setPower(0.8);

        if (slightly) { return; }

        while (slideMotor.getCurrentPosition() < 1800) {
            ez_tel(String.format("Encoder: %d", armMotor.getCurrentPosition()));
            idle();
            rainbow(0.001);
        }

        boom();
    }

    @SuppressLint("DefaultLocale")
    private void drop_the_lift(boolean slightly)
    {
        if (!activate_lift) { return; }

        if (slightly) {
            slideMotor.setTargetPosition(300);
            armMotor.setTargetPosition(600);

        } else {
            slideMotor.setTargetPosition(250);
            armMotor.setTargetPosition(100);
        }

        slideMotor.setPower(1);
        while (slideMotor.getCurrentPosition() > 1200) { idle(); rainbow(0.001); }
        armMotor.setPower(0.9);

        if (slightly) { return; }

        reverse_boom();
    }

    private void zero_motors()
    {
        if (!activate_lift) { return; }
        slideMotor.setPower(0);
        armMotor.setPower(0);
    }

    private void ez_tel(String caption)
    {
        telemetry.addData(caption, true);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectServo = hardwareMap.get(CRServo.class, "collect");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        light = hardwareMap.get(Servo.class, "light");

        colorSensor.setGain(gain);

        light.setPosition(0.280);

        double zoo = time.time();
        Trajectory scorePoint = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(artemis, Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory goToFirstSpecimen = drive.trajectoryBuilder(scorePoint.end())
                .splineToLinearHeading(new Pose2d(38.75, 21.50), Math.toRadians(0))
                .build();

        Trajectory scorePoint_2 = drive.trajectoryBuilder(goToFirstSpecimen.end())
                .splineToLinearHeading(artemis, Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory goToSecondSpecimen = drive.trajectoryBuilder(scorePoint_2.end(), true)
                .splineToLinearHeading(new Pose2d(50.0, 21.75, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory scorePoint_3 = drive.trajectoryBuilder(goToSecondSpecimen.end())
                .splineToLinearHeading(artemis, Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory goToLastSpecimen = drive.trajectoryBuilder(scorePoint_3.end(), true)
                .splineToLinearHeading(new Pose2d(61, 21.5, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory scorePoint_4 = drive.trajectoryBuilder(goToLastSpecimen.end())
                .splineToLinearHeading(artemis.plus(new Pose2d(0.75, 0.75)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        double difference = 100 * Math.abs(hardwareMap.voltageSensor.iterator().next().getVoltage() - 14.33) / 14.33;
        if (difference > 17) { telemetry.addData("Please change battery", true);
        } else if (difference > 11) { telemetry.addData("Change battery soon", true); }

        telemetry.addData("time to build: ", time.time() - zoo);
        telemetry.addData("voltage error (norm difference 2-9) %", difference);
        telemetry.update();

        light.setPosition(0.5);

        waitForStart();
        drive.update();

        if (isStopRequested()) return;

        light.setPosition(0.555);

        // drive to release a sample
        ez_tel("following 1st traj");
        drive.followTrajectory(scorePoint);

        ez_tel("lifting the arm and slide");
        lift_the_lift(false);
        zero_motors();
        ez_tel("slightly dropping the arm and slide");
        drop_the_lift(true);


        // drives to first specimen location
        ez_tel("following 2nd traj");
        drive.followTrajectory(goToFirstSpecimen);
        ez_tel("dropping the lift; intaking the specimen");
        drop_the_lift(false);


        // drives to release specimen
        ez_tel("following 3rd traj");
        drive.followTrajectory(scorePoint_2);

        ez_tel("lifting the arm and slide");
        lift_the_lift(false);
        zero_motors();
        ez_tel("slightly dropping the arm and slide");
        drop_the_lift(true);


        // drives to 2nd specimen location
        ez_tel("following the 4th traj");
        drive.followTrajectory(goToSecondSpecimen);

        ez_tel("dropping the lift; intaking the specimen");
        drop_the_lift(false);


        // drives to release specimen
        ez_tel("following 5th traj");
        drive.followTrajectory(scorePoint_3);

        ez_tel("lifting the arm and slide");
        lift_the_lift(false);
        zero_motors();
        ez_tel("slightly dropping the arm and slide");
        drop_the_lift(true);


        // drives to last specimen location
        ez_tel("following 6th traj");
        drive.followTrajectory(goToLastSpecimen);

        ez_tel("dropping the lift; intaking the specimen");
        drop_the_lift(false);

        zero_motors();

        // drives to release specimen
        ez_tel("following 7th traj");
        drive.followTrajectory(scorePoint_4);

        ez_tel("lifting the arm and slide");
        lift_the_lift(false);
        zero_motors();
        ez_tel("slightly dropping the arm and slide");
        drop_the_lift(true);

    }
}
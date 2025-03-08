package org.firstinspires.ftc.teamcode.drive.red;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.drive.Drive;
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

@Autonomous(group = "drive", name = "dontpressthiscasey")
public class gavins_dog_is_high extends LinearOpMode {

    DcMotorEx armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    Servo light = null;
    ElapsedTime time = new ElapsedTime();
    double color = 0d;
    private void rainbow(double increment)
    {
        color = light.getPosition();
        light.setPosition(color += increment);

        if (light.getPosition() > 0.722)
        {
            light.setPosition(0.280);
        }
    }
    private void ez_tel(String caption) {
        telemetry.addData(caption, true);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, 60, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectServo = hardwareMap.get(CRServo.class, "collect");

        light = hardwareMap.get(Servo.class, "light");

        light.setPosition(0.280);

        double zoo = time.time();
        Trajectory firstPoint = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-36, -52), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.33))
                .build();
        Trajectory secondPoint = drive.trajectoryBuilder(firstPoint.end())
                .splineToLinearHeading(new Pose2d(36, -52), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.33))
                .build();
        Trajectory thirdPoint = drive.trajectoryBuilder(secondPoint.end())
                .splineToLinearHeading(new Pose2d(36, 52), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.33))
                .build();
        Trajectory forthPoint = drive.trajectoryBuilder(thirdPoint.end())
                .splineToLinearHeading(new Pose2d(-36, 52), Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.33))
                .build();
        Trajectory technically_first = drive.trajectoryBuilder(forthPoint.end())
                .splineToLinearHeading(new Pose2d(-36, -52), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL * 1.33, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 1.33))
                .build();

        double difference = 100 * Math.abs(hardwareMap.voltageSensor.iterator().next().getVoltage() - 13.50) / 13.50;
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


        drive.followTrajectory(firstPoint);
        drive.followTrajectory(secondPoint);
        drive.followTrajectory(thirdPoint);
        drive.followTrajectory(forthPoint);

        for (int i = 0; i < 99; i++)
        {
            drive.followTrajectory(technically_first);
            drive.followTrajectory(secondPoint);
            drive.followTrajectory(thirdPoint);
            drive.followTrajectory(forthPoint);
            wait(1000);
        }


    }
}
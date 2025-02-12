package org.firstinspires.ftc.teamcode.drive.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "left_side_blue")
public class left_side extends LinearOpMode {

    /*
     calibrate everything when the claw is placed back on
     try to learn dx, dy values fro the limelight and degrees? and put in a formula?
     */
    final private double TILE_SIZE = 48;
    DcMotorEx armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        // Motors & servo assignment
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);

        slideMotor = hardwareMap.get(DcMotorEx.class, "slide");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectServo = hardwareMap.get(CRServo.class, "collect");
        // end

        waitForStart();
        // YOU MUST CALL THIS BEFORE TRAJECTORY GO STARTS
        drive.update();
        if (isStopRequested()) return;

        double difference = 100 * Math.abs(hardwareMap.voltageSensor.iterator().next().getVoltage() - 15.00) / 15.00;

        telemetry.addData("voltage error (norm. lvl. is 15%) %", difference);
        if (difference > 21)
        {
            telemetry.addData("Please change battery", true);
        }
        else if (difference > 17)
        {
            telemetry.addData("Change battery soon", true);
        }

        telemetry.update();

        Trajectory moveForward = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 2)
                .build();
        Trajectory moveBackwardsSlightly = drive.trajectoryBuilder(new Pose2d())
                .back(TILE_SIZE * 0.7)
                .build();
        Trajectory specificForwards = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 0.45)
                .build();
//        Trajectory first_spline = drive.trajectoryBuilder(new Pose2d())
//                .splineToConstantHeading(new Vector2d(10, 10), Math.toRadians(0))
//                .build();

        // extend arm and drop game element

        drive.turn(Math.toRadians(-10));

        armMotor.setTargetPosition(2620); // 2450
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) { telemetry.addData("arm going up", true); telemetry.update(); }

        slideMotor.setTargetPosition(1760);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
        while (slideMotor.isBusy()) { telemetry.addData("slide going up", true); telemetry.update(); }
        sleep(100); // just some delay

        drive.followTrajectory(specificForwards);

        collectServo.setPower(-0.75);
        sleep(3000);
        collectServo.setPower(0);

        drive.followTrajectory(moveBackwardsSlightly);

        // reset it

        slideMotor.setTargetPosition(0);
        slideMotor.setPower(0.9);
        while (slideMotor.isBusy()) { telemetry.addData("slide going down", true); telemetry.update(); }

        armMotor.setTargetPosition(0);
        armMotor.setPower(0.9);

        // unturn and move towards parking location

        drive.turn(Math.toRadians(-176)); // turn 90 deg clockwise
        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(-111)); // turn 45 deg clockwise
        drive.followTrajectory(moveForward);
        drive.followTrajectory(moveForward);

    }
}

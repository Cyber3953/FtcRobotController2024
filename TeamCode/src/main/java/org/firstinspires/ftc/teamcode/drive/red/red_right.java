package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "red_right")
public class red_right extends LinearOpMode {

    final private double TILE_SIZE = 48;

    DcMotor armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-24, -72, Math.toRadians(0));

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
        drive.update();

        if (isStopRequested()) return;

        double difference = 100 * Math.abs(hardwareMap.voltageSensor.iterator().next().getVoltage() - 14.77)/ 14.77;

        telemetry.addData("volt error %", difference);
        if (difference > 10)
        {
            telemetry.addData("Please change battery", true);
        }
        else if (difference > 7)
        {
            telemetry.addData("Change battery soon", true);
        }
        telemetry.update();

        Trajectory moveForward = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 2)
                .build();
        Trajectory moveBackwardsSlightly = drive.trajectoryBuilder(new Pose2d())
                .back(TILE_SIZE / 7)
                .build();
        Trajectory specificForwards = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 0.3)
                .build();
        Trajectory specificForwardsRightf = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 0.85)
                .build();

        // moves forward
        drive.followTrajectory(moveForward);
        drive.followTrajectory(specificForwardsRightf);

        // turn slightly to the left (miniscule)
        drive.turn(Math.toRadians(15));

        // essentially the same as the left auto, slightly DIFFERENT values
        armMotor.setTargetPosition(2625); // 2450
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);
        sleep(1500);

        slideMotor.setTargetPosition(1770);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.9);
        sleep(1500);

        drive.followTrajectory(specificForwards);

        collectServo.setPower(-1);
        sleep(3000);
        collectServo.setPower(0);

        drive.followTrajectory(moveBackwardsSlightly);

        // reset it

        slideMotor.setTargetPosition(0);
        slideMotor.setPower(0.9);
        sleep(1725);

        armMotor.setTargetPosition(0);
        armMotor.setPower(0.9);

        // unturn (back to forward) and move towards parking location
        // also slightly different than the left code

        drive.turn(Math.toRadians(-189)); // turn 90 deg clockwise
        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(-111)); // turn 45 deg clockwise
        drive.followTrajectory(moveForward);
        drive.followTrajectory(moveForward);

    }
}

//         //parking code, if no put in
//        drive.turn(Math.toRadians(-90));
//        drive.followTrajectory(move);
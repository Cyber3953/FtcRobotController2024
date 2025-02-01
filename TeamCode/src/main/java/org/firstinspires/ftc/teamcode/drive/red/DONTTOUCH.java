package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "DONTTOUCH")
public class DONTTOUCH extends LinearOpMode {

    /*
    * arm encoder; 2495
    * slide encoder: 1885 (limit: 1900)
    * angle degrees: 65.33
    * */
    DcMotor armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    // have to set a time for this one
    final private double TILE_SIZE = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(24, -72, Math.toRadians(0));

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

        double difference = 100 * Math.abs(hardwareMap.voltageSensor.iterator().next().getVoltage() - 15.00) / 15.00;

        telemetry.addData("voltage error %", difference);
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

        // extend arm and drop game element

        drive.turn(Math.toRadians(-10));

        armMotor.setTargetPosition(2650); // 2450
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);
        sleep(1500);

        slideMotor.setTargetPosition(1820);
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

        // unturn and move towards parking location

        drive.turn(Math.toRadians(-176)); // turn 90 deg clockwise
        drive.followTrajectory(moveForward);
        drive.turn(Math.toRadians(-111)); // turn 45 deg clockwise
        drive.followTrajectory(moveForward);
        drive.followTrajectory(moveForward);
    }
}

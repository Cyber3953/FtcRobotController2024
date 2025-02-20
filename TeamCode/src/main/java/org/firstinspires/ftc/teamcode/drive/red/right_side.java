package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "right_side_red")
public class right_side extends LinearOpMode {

    DcMotorEx armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    ElapsedTime time = new ElapsedTime();
    double zoo = 0d;

    private void lift_the_lift(boolean slightly)
    {
        if (slightly)
        {
            armMotor.setTargetPosition(2300);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(300);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else
        {
            armMotor.setTargetPosition(2700);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(1900);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        armMotor.setPower(1);
        slideMotor.setPower(0.9);

        if (slightly) { return; }

        while (slideMotor.getCurrentPosition() < 1800) { idle(); }

        collectServo.setPower(-1);
        telemetry.addData("servo is spinning", true); telemetry.update();
        sleep(2000);
        collectServo.setPower(0);
    }

    private void drop_the_lift(boolean slightly)
    {
        if (slightly)
        {
            // arbitary numbers
            slideMotor.setTargetPosition(100);
            armMotor.setTargetPosition(50);
        }
        else
        {
            // arbitary number
            slideMotor.setTargetPosition(0);
            armMotor.setTargetPosition(0);
        }

        slideMotor.setPower(1);
        while (slideMotor.getCurrentPosition() > 1500) { idle(); }
        armMotor.setPower(0.9);

        if (slightly) { return; }

        collectServo.setPower(1);
        telemetry.addData("servo is spinning", true); telemetry.update();
        sleep(1500);
        collectServo.setPower(0);
    }

    private void zero_motors()
    {
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

        zoo = time.time();
        Trajectory foo1 = drive.trajectoryBuilder(startPose)
                .splineToSplineHeading(new Pose2d(49.5, 47.0, Math.toRadians(45)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory foo1_1 = drive.trajectoryBuilder(foo1.end(), true)
                .splineTo(new Vector2d(25, 45), Math.toRadians(180))
                .addTemporalMarker(1, () -> {
                    zero_motors();
                })
                .build();

        Trajectory foo2 = drive.trajectoryBuilder(foo1_1.end())
                .splineToSplineHeading(new Pose2d(38.2, 22.0, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40.0 ,DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        Trajectory foo3 = drive.trajectoryBuilder(foo2.end())
                .splineToSplineHeading(new Pose2d(49.5, 47.0, Math.toRadians(45)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory foo4 = drive.trajectoryBuilder(foo3.end()) // maybe decrease this y??
                .splineToSplineHeading(new Pose2d(48.0, 21.5, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1, () -> {
                    zero_motors();
                })
                .build();

        Trajectory foo4_1  = drive.trajectoryBuilder(foo4.end(), true)
                .splineTo(new Vector2d(41.2, 22.0), Math.toRadians(0))
                .build();


        Trajectory foo5 = drive.trajectoryBuilder(foo4.end())
                .splineToSplineHeading(new Pose2d(49.5, 47.0, Math.toRadians(45)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    lift_the_lift(true);
                })
                .build();

        Trajectory parking = drive.trajectoryBuilder(foo5.end())
                .splineToSplineHeading(new Pose2d(24, 0, Math.toRadians(0)), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    // arbitrary number??
                    armMotor.setTargetPosition(500);
                    armMotor.setPower(0.5);
                })
                .build();

        telemetry.addData("time to build: ", time.time() - zoo);
        telemetry.update();

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


        // drive to release a sample
        drive.followTrajectory(foo1);
        ez_tel("following 1st traj");

        lift_the_lift(false);
        ez_tel("lifting the arm and slide");
        zero_motors();
        drop_the_lift(true);
        ez_tel("slightly dropping the arm and slide");


        // drives slightly back
        drive.followTrajectory(foo1_1);
        ez_tel("going back slightly");

        // drives to first specimen location
        drive.followTrajectory(foo2);
        ez_tel("following 2nd traj");
        drop_the_lift(false);
        ez_tel("dropping the lift; intaking the specimen");


        // drives to release specimen
        drive.followTrajectory(foo3);
        ez_tel("following 3rd traj");

        lift_the_lift(false);
        ez_tel("lifting the arm and slide");
        zero_motors();
        drop_the_lift(true);
        ez_tel("slightly dropping the arm and slide");


        // drives to 2nd specimen location
        drive.followTrajectory(foo4);
        ez_tel("following the 4th traj");

        drop_the_lift(false);
        ez_tel("dropping the lift; intaking the specimen");
        zero_motors();
        drive.followTrajectory(foo4_1);


        // drives to release specimen
        drive.followTrajectory(foo5);
        ez_tel("following 5th traj");

        lift_the_lift(false);
        ez_tel("lifting the arm and slide");
        drop_the_lift(true);
        ez_tel("slightly dropping the arm and slide");

        drive.followTrajectory(parking);

        /*
        NEVER USE DROP_THE_LIFT IN TRAJS
         */
    }
}

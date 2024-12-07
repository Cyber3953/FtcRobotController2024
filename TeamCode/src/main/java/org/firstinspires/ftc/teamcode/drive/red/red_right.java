package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "red_right")
public class red_right extends LinearOpMode {

    final private double TILE_SIZE = 42;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        // YOU MUST CALL THIS BEFORE TRAJECTORY GO STARTS
        drive.update();
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .forward(10)
                .build();

        drive.followTrajectory(traj);

        sleep(100);

    }
}
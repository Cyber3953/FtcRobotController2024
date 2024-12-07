package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "red_left")
public class red_left extends LinearOpMode {

    final private double TILE_SIZE = 42;

    @Override
    public void runOpMode() throws InterruptedException {
       SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

       waitForStart();
       // YOU MUST CALL THIS BEFORE TRAJECTORY GO STARTS
       drive.update();
       if (isStopRequested()) return;

       Trajectory traj = drive.trajectoryBuilder(new Pose2d())
               .forward(TILE_SIZE * 2)
               .build();

       drive.followTrajectory(traj);
    }
}

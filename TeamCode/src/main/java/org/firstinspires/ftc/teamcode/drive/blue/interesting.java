package org.firstinspires.ftc.teamcode.drive.blue;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "circle")
public class interesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        Pose2d startPose = new Pose2d(24, -24, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        drive.update();
        if (isStopRequested()) return;
        Trajectory my_traj = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(120, 0), 0)
                .build();

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
//                .splineTo(new Vector2d(0.0, 48.0), 0.0 )
//                .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
//                .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
//                .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
//                .build();

        drive.followTrajectory(my_traj);

        sleep(500);

        drive.followTrajectory(
                drive.trajectoryBuilder(my_traj.end(), true)
                .splineTo(new Vector2d(0,0), 0)
                .build());

    }
}

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

        drive.update();
        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0.0, 48.0), 0.0 )
                .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
                .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
                .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                .build();

        drive.followTrajectory(traj);

    }
}

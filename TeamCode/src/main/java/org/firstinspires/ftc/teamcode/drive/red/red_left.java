package org.firstinspires.ftc.teamcode.drive.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "red_left")
public class red_left extends LinearOpMode {

    /*
    * arm encoder; 2495
    * slide encoder: 1885 (limit: 1900)
    * angle degrees: 65.33
    * */
    DcMotor armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    // have to set a time for this one
    final private double TILE_SIZE = 42;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .forward(TILE_SIZE * 2)
                .build();

        drive.followTrajectory(traj);

        drive.turn(Math.toRadians(90)); // or 180

        armMotor.setTargetPosition(2495);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);

        sleep(3000);

        slideMotor.setTargetPosition(1885);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);
        sleep(3000);

        collectServo.setPower(1);
        sleep(2000);

    }
}

package org.firstinspires.ftc.teamcode.lemonlight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "lemonlight_test")
public class lemonlight_op extends LinearOpMode {

  Pose2d startPose = new Pose2d(61, 36, Math.toRadians(0));

  @Override
  public void runOpMode() {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    drive.setPoseEstimate(startPose);

    waitForStart();

    while (!isStopRequested)
    {
      drive.update();
    }
  }
}


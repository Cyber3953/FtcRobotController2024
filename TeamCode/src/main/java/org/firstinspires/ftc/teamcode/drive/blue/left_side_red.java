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

@Autonomous(group = "drive", name = "left_side_red")
public class left_side_red extends LinearOpMode {

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

        Pose2d startPose = new Pose2d(61, 36, Math.toRadians(0));

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

        // math.toradians(180) is arbitary
        Trajectory foo1 = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-20, 12), Math.toRadians(90))
                .addTemporalMarker(1, () -> {
                    armMotor.setTargetPosition(2620); // 2450
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armMotor.setPower(1);
                    while (armMotor.isBusy()) { telemetry.addData("arm going up", true); telemetry.update(); }

                    slideMotor.setTargetPosition(1760);
                    slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotor.setPower(1);
                    while (slideMotor.isBusy()) { telemetry.addData("slide going up", true); telemetry.update(); }
                })
                .build();

        Trajectory foo2 = drive.trajectoryBuilder(foo1.end(), false)
                .splineToLinearHeading(new Pose2d(-18, 14), Math.toRadians(90))
                .addTemporalMarker(0.1, () -> {
                    // arbitary number
                    slideMotor.setTargetPosition(100);
                    slideMotor.setPower(1);
                    while (slideMotor.isBusy()) { telemetry.addData("slide going down", true); telemetry.update(); }
                    sleep(100); // just some delay

                    // arbitary number
                    armMotor.setTargetPosition(100);
                    armMotor.setPower(1);
                    while (armMotor.isBusy()) { telemetry.addData("arm going down", true); telemetry.update(); }
                })
                .build();

       drive.followTrajectory(foo1);
       sleep(100);
       drive.followTrajectory(foo2);

    }
}

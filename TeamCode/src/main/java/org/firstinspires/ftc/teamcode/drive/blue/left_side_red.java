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

    DcMotorEx armMotor = null;
    DcMotorEx slideMotor = null;
    CRServo collectServo = null;
    double foo = 100;

    private void lift_the_lift()
    {
        armMotor.setTargetPosition(2620); // 2450
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) { telemetry.addData("arm going up", true); telemetry.update(); }

        slideMotor.setTargetPosition(1760);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
        while (slideMotor.isBusy()) { telemetry.addData("slide going up", true); telemetry.update(); }
    }

    private void drop_the_lift(boolean slightly)
    {
        if (slightly) 
        {
            foo = 500;
        }
            // arbitary number
        slideMotor.setTargetPosition(slightly);
        slideMotor.setPower(1);
        while (slideMotor.isBusy()) { telemetry.addData("slide going down", true); telemetry.update(); }

        // arbitary number
        armMotor.setTargetPosition(slightly);
        armMotor.setPower(1);
        while (armMotor.isBusy()) { telemetry.addData("arm going down", true); telemetry.update(); }

        if (slightly) { return; }
        
        servo.setPower(1);
        telemetry.addData("servo is spinning", true); telemetry.update();
        sleep(2000);
        servo.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, 60, Math.toRadians(-90));

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

        Trajectory foo1 = drive.trajectoryBuilder(startPose))
                .splineTo(new Vector2d(48.471, 46.988), Math.toRadians(45))
                .build();

        Trajectory foo2 = drive.trajectoryBuilder(foo1.end(), true)
                .splineTo(new Vector2d(43.86, 25.307), Math.toRadians(0))
                .build();

        Trajectory foo3 = drive.trajectoryBuilder(foo2.end()))
                .splineTo(new Vector2d(48.471, 46.988), Math.toRadians(45))
                .build();

        drive.followTrajectory(foo1);

        robot.dropServo();
        lift_the_lift(false);
        sleep(50);
        drop_the_lift(true);
        sleep(50);

        drive.followTrajectory(foo2);
    }
}

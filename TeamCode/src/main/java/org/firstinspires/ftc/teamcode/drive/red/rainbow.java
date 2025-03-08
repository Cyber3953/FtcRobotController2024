package org.firstinspires.ftc.teamcode.drive.red;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive", name = "rainbow")
public class rainbow extends LinearOpMode {

    Servo light = null;
    double color = 0d;
    private void rainbow(double increment)
    {
        color = light.getPosition();
        light.setPosition(color += increment);

        if (light.getPosition() > 0.722)
        {
            light.setPosition(0.280);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        light = hardwareMap.get(Servo.class, "light");

        for (int i = 0; i < 1_000_000; i++)
        {
            wait(100);
            rainbow(0.001);
        }
    }
}
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

@TeleOp(name = "lemonlight")
public class lemonlight_op extends OpMode {

  static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
          = RevHubOrientationOnRobot.LogoFacingDirection.values();
  static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
          = RevHubOrientationOnRobot.UsbFacingDirection.values();

  private Limelight3A lemonlight = null;
  private Pose3D botpose;
  private YawPitchRollAngles orientation;
  private IMU imu;

  @Override
  public void init()
  {
    lemonlight = hardwareMap.get(Limelight3A.class, "LemonLime");
    imu = hardwareMap.get(IMU.class, "imu");

    updateOrientation();
    
    lemonlight.start();
    lemonlight.setPollRateHz(100);
    lemonlight.pipelineSwitch(4);
  }

  @Override
  public void loop()
  {
    // get yaw from imu for mt2
    orientation = imu.getRobotYawPitchRollAngles();
    lemonlight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
    telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));

    LLResult result = lemonlight.getLatestResult();

    if (result != null && result.isValid())
    {
      botpose = result.getBotpose_MT2();
      telemetry.addData("tx", result.getTx());
      telemetry.addData("ty", result.getTy());

      if (botpose != null) {
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        telemetry.addData("MT2 Location", "(" + x + ", " + y + ")");
      }
    }
    // calibrate x & y axis after limelight is placed on robot
  }

  void updateOrientation() {
    RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[0];
    RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[2];
    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
    imu.initialize(new IMU.Parameters(orientationOnRobot));
  }

}


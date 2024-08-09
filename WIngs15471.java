package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "WIngs15471 (Blocks to Java)", group = "")
public class WIngs15471 extends LinearOpMode 
{

  private DcMotor rightDrive;
  private DcMotor leftDrive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    rightDrive = hardwareMap.dcMotor.get("rightDrive");
    leftDrive = hardwareMap.dcMotor.get("leftDrive");

    // Put initialization blocks here.
    rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad2.a) {
        }
        if (gamepad2.b) {
        }
        if (gamepad2.y) {
        }
        leftDrive.setPower(gamepad1.left_stick_y);
        rightDrive.setPower(gamepad1.right_stick_y);
        telemetry.update();
      }
    }
  }
}

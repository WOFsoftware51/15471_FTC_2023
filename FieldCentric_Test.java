package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp
public class FieldCentric extends LinearOpMode 
{
    IMU imu;
    String hippo = "hippo";
    YawPitchRollAngles myRobotOrientation;
    
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    // DcMotor motorC2;
    // DcMotor motorC3;
    // DcMotor motor2;
    // DcMotor motor3;
    // Servo servoMotor0;
    // Servo servoMotor1;
    // Servo servoMotor2;
    // Servo servoMotor3;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        // motorC2= hardwareMap.dcMotor.get("motorC2");
        // motorC3 = hardwareMap.dcMotor.get("motorC3");
        // motor2 = hardwareMap.dcMotor.get("motor2");
        // motor3 = hardwareMap.dcMotor.get("motor3");
        // servoMotor0 = hardwareMap.servo.get("servo0");
        // servoMotor1 = hardwareMap.servo.get("servo1");
        // servoMotor2 = hardwareMap.servo.get("servo2");
        // servoMotor3 = hardwareMap.servo.get("servo3");

        imu = hardwareMap.get(IMU.class, "imu");
                    
        double ARM_SPEED = 10;
        int init_timer = 0;
        
        double yaw = 0;
        double pitch = 0;
        double roll = 0;
        String centric;
        double yp;
        double xp;


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //motorArm.setZeroPowerBehavior(BRAKE);
        
        


        boolean slow = false;
        int isFieldCentric = 1;
        double toggle_drive = 0;
        int timer = 0;
        imu.resetYaw();
        init_Orientation();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            
            // Initialization section Because I don't know how to do an initialization section in any other way
            // if(init_timer == 0){
            //     init_timer++;
            // }
            myRobotOrientation = imu.getRobotYawPitchRollAngles();
            
            // imu.initialize(myIMUparameters);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            
            yaw   = myRobotOrientation.getYaw(AngleUnit.DEGREES);
            pitch = myRobotOrientation.getPitch(AngleUnit.DEGREES);
            roll  = myRobotOrientation.getRoll(AngleUnit.DEGREES);
            
            ////////////////////////////////////////////////////////////////////
            
            
            
            //////////////
            if(gamepad1.right_bumper)
            {
                slow = true;
            }
            else
            {
                slow = false;
            }
            
            ///////////////////////////////////////////////////////////////////////////////
            
            // if(gamepad1.left_bumper)
            // {
            //     if(toggle_drive == 0){
            //         isFieldCentric = (isFieldCentric + 1) % 2;
            //         timer++;
            //     }
            // }
            // else{
            //     toggle_drive = 0;
            // }
            
            if(gamepad1.start){
                isFieldCentric = 1;
            
            }
            
            if(gamepad1.back){
                isFieldCentric = 0;
            
            }

            
            ///////////////////////////////////////////////////////////////////////////////
            
            if(gamepad1.y)
            {
                imu.resetYaw();
            }
            
            //////////////////////////////////////////////////////////////////////////////
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            
            //y prime and x prime are new 
            if(isFieldCentric==1){
                 yp = (y*cos(yaw) - x*sin(yaw));
                 xp = (x*cos(yaw) + y*sin(yaw));
            }
            else{
                yp = y;
                xp = x;
            }
            
            
            double denominator = Math.max(Math.abs(yp) + Math.abs(xp) + Math.abs(rx), 1);
            double frontLeftPower = (yp + xp + rx) / denominator;
            double backLeftPower = (yp - xp + rx) / denominator;
            double frontRightPower = (yp - xp - rx) / denominator;
            double backRightPower = (yp + xp - rx) / denominator;
            

            if(slow)
            {
                frontLeftPower = frontLeftPower*0.5;
                backLeftPower = backLeftPower*0.5;
                frontRightPower = frontRightPower*0.5;
                backRightPower = backRightPower*0.5;
            }
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(-backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(-backRightPower);

            
            if(isFieldCentric==1){
                centric = "Field";
            }
            else{
                centric = "Robot";
            }
            
            telemetry.addData("Controller 2 Left Stick Y Pos", gamepad2.left_stick_y);
         //   telemetry.addData("Y axis", yaw);
            telemetry.addData("X axis", pitch);
            telemetry.addData("Z axis", roll);
            telemetry.addData(centric, "Centric");
            telemetry.addData("Yaw", yaw);
         

            
            telemetry.update();
            
                        

            
        }
        
        
    }
    
        public void init_Orientation(){            
            RevHubOrientationOnRobot orientationOnRobot = 
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }        
        public double cos(double degrees){
            return Math.cos(Math.toRadians(degrees));
        }
        public double sin(double degrees){
            return Math.sin(Math.toRadians(degrees));
        }
        
}

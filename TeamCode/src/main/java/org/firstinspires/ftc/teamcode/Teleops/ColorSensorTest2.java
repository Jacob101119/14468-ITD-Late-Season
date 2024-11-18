package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;

@TeleOp
public class ColorSensorTest2 extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();
        //USE SPEED VARIABLE FOR DRIVE
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            robot.TeleopUpdate();



            //robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x));





            String detectedColor = robot.detectColor();

            if (detectedColor.equals("BlueSample")) {
                telemetry.addLine("BLUE SAMPLE DETECTED");
                telemetry.update();
            }
            if(detectedColor.equals("YellowSample")){
                telemetry.addLine("YELLOW SAMPLE DETECTED");
                telemetry.update();
            }
            if(detectedColor.equals("RedSample")){
                telemetry.addLine("RED SAMPLE DETECTED");
                telemetry.update();
            }
            if(detectedColor.equals("No Color")){
                telemetry.addLine("No color detected");
                telemetry.update();
            }
        }
    }
}


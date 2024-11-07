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
public class FieldCentric extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;


    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();


        robot = new BaseRobot(hardwareMap);

        while (!isStopRequested() && opModeIsActive()) {


            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));



            drive.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x, .5, telemetry);
            telemetry.addData("Orientation", Math.toDegrees(drive.getHeading()));
            telemetry.update();


            if (gamepad1.start) {
                drive.resetHeading();
            }
            telemetry.addLine("gamepad1.start: Reset heading");


        }


    }
}
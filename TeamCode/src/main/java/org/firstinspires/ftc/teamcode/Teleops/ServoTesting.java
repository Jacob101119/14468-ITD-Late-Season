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
public class ServoTesting extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


            //updates

            double stickSpeed = .05;
            double speed = .005;
            robot.update();
            //end


            if (gamepad2.a){
                robot.setOuttakeWristPos(robot.getV4B_IN_ROBOT());
            }
            if(gamepad2.y){
                robot.setOuttakeWristPos(robot.getV4B_INTAKE_POS());
            }
            robot.changeAxlePos(gamepad2.right_stick_y * stickSpeed);
            robot.changeV4bPos(gamepad2.left_stick_y * stickSpeed);
            robot.changeIntakeGrasperPos(gamepad1.left_stick_y * stickSpeed);
            robot.changeOuttakeGrasperPos(gamepad1.right_stick_y * stickSpeed);

            if (gamepad1.y){
                robot.changeWristPos(speed);
            }
            if (gamepad1.a){
                robot.changeWristPos(-speed);
            }


            telemetry.addData("1 - Intake Grasper - left Stick: ", robot.getIntakeGrasperPos());
            telemetry.addLine();
            telemetry.addData("1 - Outtake Grasper - right stick", robot.getOuttakeGrasperPos());
            telemetry.addLine();
            telemetry.addData("2 - V4b - left stick", robot.getV4bPos());
            telemetry.addLine();
            telemetry.addData("2 - axle - right stick", robot.getOuttakeAxlePos());
            telemetry.addLine();
            telemetry.addData("2 - wrist - y/a", robot.getOuttakeWristPos());

            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}


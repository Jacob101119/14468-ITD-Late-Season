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
        //robot.setV4bPos(robot.getV4B_INTAKE_POS());
        //robot.setAxlePos(robot.getAXLE_HB());


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            //robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


                    //.1482 axle straight out

            double stickSpeed = .01;
            double speed = .005;
            robot.update();
            //end


            //axle down: .9773
            //HB axle: .35
            //axle out: .181
            if (gamepad2.a){
                robot.setV4bPos(robot.getV4B_IN_ROBOT());
            }
            if(gamepad2.y){
               robot.setV4bPos(robot.getV4B_INTAKE_POS());
            }
            robot.changeAxlePos(gamepad2.right_stick_y * stickSpeed);
            robot.changeV4bPos(gamepad2.left_stick_y * stickSpeed);
            robot.changeGimbalPos(gamepad1.left_stick_y * stickSpeed);
            robot.changeIntakeGrasperPos(gamepad1.right_stick_y * stickSpeed);

            if(gamepad2.y){
                //robot.changeGimbalPos(.01);
                robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
            }if(gamepad2.a){
                //robot.changeGimbalPos(-.01);
                robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
            }


            //tray open: .4743
            //tray closed: .0461
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
            telemetry.addData("intake Gimbal - 2y and a", robot.getGimbalPos());
            telemetry.addData("tray pos", robot.getTrayPos());

            //intake claw open: .29
            //intake claw closed: .54

            //intake gimbal reset" .2
            //v4b down: 0
            //v4b to tray: .6186

            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}


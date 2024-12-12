package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class TestingTeleop extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            //robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


            //updates

            robot.update();
            //end





            if(gamepad1.a){
                robot.setV4bPos(Constants.v4bConstants.ground);
            }
            if(gamepad1.y){
                robot.setV4bPos(Constants.v4bConstants.up);
            }

            if(gamepad1.dpad_up){
                //robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TRANSFER());

            }

            if(gamepad1.x){
                //robot.setTrayPos(robot.getTRAY_OPEN());
            }
            if(gamepad1.b){
                //robot.setTrayPos(robot.getTRAY_CLOSED());
            }
            if(gamepad1.dpad_down){
                //robot.setAxlePos(robot.getAXLE_DOWN());
            }
            if(gamepad1.dpad_left){
                //robot.setAxlePos(robot.getAXLE_PASS_THROUGH());
            }




                        //telemetry
            //_____________________________________________________________________________________


            telemetry.addData("joystick", gamepad1.right_stick_y);

            telemetry.addLine("robot position (starting at x: 0, y: 0, heading: 0)");
            telemetry.addData("x:", drive.pose.position.x);
            telemetry.addData("y:", drive.pose.position.y);
            telemetry.addData("heading (deg):", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("Motors: ");
            telemetry.addLine();

            telemetry.addLine("Slides: ");
            telemetry.addData("outtake slides position: ", robot.getRightOuttakeSlidePos());
            //telemetry.addData("outtake slides power: ", robot.getOUTTAKE_SLIDES_POWER());
            telemetry.addLine();

            telemetry.addData("intake slides position: ", robot.getIntakeSlidesPos());
            //telemetry.addData("intake slides power: ", robot.getINTAKE_SLIDES_POWER());
            telemetry.addLine();
            telemetry.addLine();

            //telemetry.addData("hang arm position: ", robot.getHangArmPos());




            telemetry.addLine();
            telemetry.addLine("Servos: ");
            telemetry.addLine();

            telemetry.addLine("Intake:");
            telemetry.addData("gimbal servo pos" , robot.getGimbalPos());
            telemetry.addData("intake grasper pos" , robot.getIntakeGrasperPos());
            telemetry.addData("v4b pos" , robot.getV4bPos());
            telemetry.addLine();

            telemetry.addLine("Outtake:");
            telemetry.addData("Axle servo pos" , robot.getOuttakeAxlePos());
            telemetry.addData("outtake wrist pos:" , robot.getOuttakeWristPos());
            telemetry.addData("outtake grasper pos" , robot.getOuttakeGrasperPos());

            telemetry.addLine();
            telemetry.addLine();

            /*
            telemetry.addLine("controls: ");
            telemetry.addLine();

            telemetry.addLine("Gamepad1:");
            telemetry.addLine("right/left stick: drive");
            //telemetry.addLine("right/left trigger: slides");
            telemetry.addLine();

            telemetry.addLine("Gamepad2:");
            telemetry.addLine("left stick: pivot motor");
            telemetry.addLine("right stick: slides");
            telemetry.addLine("dpad_up/down: axle rotation servo");
            telemetry.addLine("dpad_left/right: grasper");
            telemetry.addLine("a: pivot motor back");
            telemetry.addLine("y: pivot motor vertical");
            telemetry.addLine("b: pivot motor horizontal");
            telemetry.addLine("left/right bumper: change gimbal pos");
            telemetry.addLine("left&right bumper at the same time: reset gimbal pos");
             */
            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}


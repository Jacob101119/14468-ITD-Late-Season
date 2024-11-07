package org.firstinspires.ftc.teamcode.Teleops.NetZone;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;

@TeleOp
public class OZ_Teleop_V1 extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();
        //USE SPEED VARIABLE FOR DRIVE
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){

            double driveSpeed = 1;
            robot.TeleopUpdate();

            //GAMEPAD1------------------------------------------------------------------------

            //drive
            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed), -gamepad1.right_stick_x * driveSpeed));

            //drive speeds
            if (gamepad1.dpad_up){
                driveSpeed = 1;//full speed
            }
            if (gamepad1.dpad_left || gamepad1.dpad_right){
                driveSpeed = .5;//half speed
            }
            if (gamepad1.dpad_down){
                driveSpeed = 0.25;//quarter speed
            }
            //end drive speeds
            //end drive

            //intake slides

            double intakePower = gamepad1.right_trigger-gamepad1.left_trigger;
            if (Math.abs(intakePower)> 0.1) {
                robot.setIntakePower(intakePower);
            }
            else {
                robot.updateIntakeSlidesPos();
            }//end intake slides


            //gimbal
            if (gamepad1.left_bumper){
                robot.changeGimbalPos(-.005);
            }
            if (gamepad1.right_bumper){
                robot.changeGimbalPos(.005);
            }
            //end gimbal

            //intake claw
            if(gamepad1.x){
                robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_OPEN());
            }
            if(gamepad1.b){
                robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_CLOSED());
            }
            //end intake claw

            //intake action
            if(gamepad1.y){
                //intake action
            }
            //end intake action

            //intake position
            if (gamepad1.a){
                robot.intakingFromGround();//v4b down, claw open, gimbal reset
            }
            //end intake position


            //GAMEPAD2------------------------------------------------------------------------


            //outtake slides
            double outtakePower = gamepad2.right_trigger-gamepad2.left_trigger;
            if (Math.abs(outtakePower) > 0.1) {
                robot.setOuttakePower(outtakePower);
            }
            else {
                robot.updateOuttakeSlidesPos();
            }

            //presets
            if (gamepad2.dpad_left){
                robot.setAxlePos(robot.getAXLE_TO_TRAY());
                robot.setOuttakeWristPos(robot.getWRIST_TO_TRAY());
            }
            if (gamepad2.dpad_right){
                robot.setAxlePos(robot.getAXLE_TO_WALL());
                robot.setOuttakeWristPos(robot.getOUTTAKE_WRIST_TO_WALL());
            }
            if(gamepad2.dpad_up){
                robot.SpecimenScoring();
            }
            if(gamepad2.dpad_down){
                robot.resetOuttake();
            }
            if(gamepad2.y){
                robot.HighBucketScoring();
            }
            //end presets

            //claw
            if(gamepad2.right_bumper){
                robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
            }
            if(gamepad2.left_bumper){
                robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
            }











            //telemetry
            //_____________________________________________________________________________________



            telemetry.addLine("robot position (starting at x: 0, y: 0, heading: 0)");
            telemetry.addData("x:", drive.pose.position.x);
            telemetry.addData("y:", drive.pose.position.y);
            telemetry.addData("heading (deg):", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("Motors: ");
            telemetry.addLine();

            telemetry.addLine("Slides: ");
            telemetry.addData("outtake slides position: ", robot.getOuttakeSlidesPos());
            telemetry.addData("outtake slides power: ", robot.getOUTTAKE_SLIDES_POWER());
            telemetry.addLine();

            telemetry.addData("intake slides position: ", robot.getIntakeSlidesPos());
            telemetry.addData("intake slides power: ", robot.getINTAKE_SLIDES_POWER());
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


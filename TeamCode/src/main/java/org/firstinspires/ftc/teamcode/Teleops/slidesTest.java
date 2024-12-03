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
public class slidesTest extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;



    @Override
    public void runOpMode() throws InterruptedException {




        waitForStart();
        //USE SPEED VARIABLE FOR DRIVE

        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){


            robot.TeleopUpdate();

            //GAMEPAD1------------------------------------------------------------------------


            double outtakePower = gamepad2.right_trigger-gamepad2.left_trigger;
            if (Math.abs(outtakePower) > 0.1) {
                robot.setOuttakePower(outtakePower);
            }
            else {
                robot.updateOuttakeSlidesPos();
            }


            //SPECIMEN______________________________________________________________
            //presets












            //telemetry
            //_____________________________________________________________________________________


           /* double voltage = drive.voltageSensor.getVoltage();
            if(voltage < 12 && voltage > 10){
                telemetry.addLine("MEDIUM VOLTAGE");
                telemetry.update();
            }
            if(voltage >12){
                telemetry.addLine("Good voltage");
            }
            if(voltage <10){
                telemetry.addLine("LOW VOLTAGE");
                telemetry.update();
            }

            */




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


            telemetry.addLine("controls: ");
            telemetry.addLine();

            telemetry.addLine("Gamepad1:");
            telemetry.addLine("right/left stick: drive");
            telemetry.addLine("dpad_up/right & left/down: speeds high/med/low");

            telemetry.addLine();

            telemetry.addLine("Gamepad2:");
            telemetry.addLine("sticks: intake slides");
            telemetry.addLine("triggers: outtake slides");
            telemetry.addLine("dpad_up: Specimen scoring");
            telemetry.addLine("dpad_left: HB scoring");
            telemetry.addLine("dpad_down: Grab from wall");
            telemetry.addLine("dpad_right: v4b hover over ground");
            telemetry.addLine("bumpers: outtake claw");
            telemetry.addLine("b/x: intake claw open/closed");
            telemetry.addLine("y: intake action");
            telemetry.addLine("a: v4b intake pos");


            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}


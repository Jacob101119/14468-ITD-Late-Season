package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class old_teleop extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    volatile BaseRobot robot;
    private volatile boolean endTeleop = false;



    @Override
    public void runOpMode() throws InterruptedException {


        Thread driveThread = new Thread(this::driveLoop);



        waitForStart();
        //USE SPEED VARIABLE FOR DRIVE
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);
        driveThread.start();

        while(!isStopRequested() && opModeIsActive()){

            robot.TeleopUpdate();

            //drive speeds



            //if(gamepad2.a){
              //  Actions.runBlocking(new TransferAction(robot));
            //}

            //end drive speeds
            //end drive

            //intake slides

            double intakePower = gamepad2.right_stick_y-gamepad2.left_stick_y *.3;
            if (Math.abs(intakePower)> 0.1) {
                robot.setIntakePower(intakePower);
            }
            else {
                robot.updateIntakeSlidesPos();

            }//end intake slides


            //gimbal
            if (gamepad2.back){
                robot.changeGimbalPos(-.005);
            }
            if (gamepad2.start){
                robot.changeGimbalPos(.005);
            }

            if(gamepad2.right_stick_button || gamepad2.left_stick_button){
                robot.setGimbalPos(Constants.intakeClawConstants.gimbalReset);
            }
            //end gimbal





            //GAMEPAD2------------------------------------------------------------------------


            //outtake slides
            double outtakePower = gamepad2.right_trigger-gamepad2.left_trigger;
            if (Math.abs(outtakePower) > 0.1) {
                robot.setOuttakePower(outtakePower);
            }
            else {
                robot.updateOuttakeSlidesPos();
            }


            //SPECIMEN______________________________________________________________
            //presets

            if (gamepad2.dpad_left){
                robot.HighBucketScoring();//high bucket
            }
            if(gamepad2.dpad_up){
                robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
                robot.setV4bPos(Constants.v4bConstants.up);
            }
            if(gamepad2.dpad_down){
                robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);//grab from wall
                robot.setOuttakeSlidesPos(0);
                //robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
            }

            //end presets

            //claw
            if(gamepad2.right_bumper){
                robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
            }
            if(gamepad2.left_bumper){
                robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
            }
            //END SPECIMEN_________________________________

            //TRANSFER______________________________


            if(gamepad2.a){
                robot.intakeGround();//get ready for intake
            }
            if(gamepad2.y){
                robot.setAxlePos(Constants.v4bConstants.up);
                robot.setIntakeSlidesPos(0);
            }
            if(gamepad2.dpad_right){
                robot.prepForIntake();
            }


            //claw
            if(gamepad2.x){
                robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);//close claw
            }
            if(gamepad2.b){
                robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);//open claw
            }


            telemetry.addLine("robot position (starting at x: 0, y: 0, heading: 0)");
            telemetry.addData("x:", drive.pose.position.x);
            telemetry.addData("y:", drive.pose.position.y);
            telemetry.addData("heading (deg):", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addLine();
            telemetry.addLine();


            telemetry.addLine("Motors: ");
            telemetry.addLine();

            telemetry.addLine("Slides: ");
            //telemetry.addData("outtake slides position: ", robot.getOuttakeSlidesPos());
            //telemetry.addData("outtake slides power: ", robot.getOUTTAKE_SLIDES_POWER());
            telemetry.addLine();

            telemetry.addData("intake slides position: ", robot.getRightIntakeSlidePos());
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
        endTeleop=true;
        try{
            driveThread.join();
        }
        catch (InterruptedException e){

        }

    }



    public void driveLoop(){
        while(!endTeleop){

            double driveSpeed = 1;


            //GAMEPAD1------------------------------------------------------------------------

            if (gamepad1.dpad_up){
                driveSpeed = 1;//full speed
            }
            if (gamepad1.dpad_left || gamepad1.dpad_right){
                driveSpeed = .5;//half speed
            }
            if (gamepad1.dpad_down){
                driveSpeed = 0.25;//quarter speed
            }
            if(gamepad1.a){
                driveSpeed = .125;//1/8 speed
            }
            //drive
            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed), -gamepad1.right_stick_x * driveSpeed));

        }
    }
}


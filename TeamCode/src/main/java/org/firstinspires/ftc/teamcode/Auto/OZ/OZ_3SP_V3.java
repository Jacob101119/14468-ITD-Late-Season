package org.firstinspires.ftc.teamcode.Auto.OZ;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;




@Autonomous
public final class OZ_3SP_V3 extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        robot = new BaseRobot(hardwareMap, new Pose2d(12, -62.91, Math.toRadians(-90)));



        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());



        //test heading reset
        robot.drive.resetHeading();

        waitForStart();


        robot.setAxlePos(robot.getAXLE_TO_WALL());
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());//slides above high chamber
        robot.update();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.2, -32.3))
                .build();
        Actions.runBlocking(moveToSub);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        robot.update();

        robot.delay(.5);//was .8

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());

        robot.update();
        robot.delay(.3);
        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action moveToSample = robot.drive.actionBuilder(robot.drive.pose)
                //SPLINE TO CONSTANT HEADING
                //the tangent is like the angle of approach
                //going straight to the right - 0
                //straight to left - 180
                //straight back - -270
                //straight forward - 90


                .splineToConstantHeading(new Vector2d(12, -45), 270)//move back from sub //was (12, -41)
                .splineToConstantHeading(new Vector2d(32.9, -45), 0)//sideways //was (32.9, -41)
                .splineToConstantHeading(new Vector2d(32.9, -18), 90)//forward
                .strafeToLinearHeading(new Vector2d(41, -14), Math.toRadians(0))//move to sample
                .strafeToConstantHeading(new Vector2d(41, -62))//move to OZ
                .strafeToLinearHeading(new Vector2d(41, -58), Math.toRadians(90))//turn to grab spec
                .splineToConstantHeading(new Vector2d(41, -62),90)
                .build();
        Actions.runBlocking(moveToSample);



        robot.delay(.2);//was 1

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        robot.update();

        robot.delay(.2);//was 1

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        robot.update();

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(41, -55))//move back
                .strafeToLinearHeading(new Vector2d(4, -30.8), Math.toRadians(-90))//move to sub
                .build();
        Actions.runBlocking(moveToSub2);


        robot.update();

        //robot.delay(.9);



        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        robot.update();
        robot.delay(.4);//was .5

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        //robot.delay(.3);

        robot.setOuttakeSlidesPos(0);
        robot.update();


        Action moveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -48))

                .strafeToLinearHeading(new Vector2d(38, -59), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(38, -64.5))
                .build();
        Actions.runBlocking(moveToOZ);
        robot.delay(.2);//was 1

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        robot.update();

        robot.delay(.2);//was 1
//EDITS DONE HERE

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        robot.update();

        Action moveToSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(38, -59))
                .strafeToLinearHeading(new Vector2d(4, -36), Math.toRadians(-90))
                .build();
        Actions.runBlocking(moveToSub3);



        //robot.delay(.5);

       // Action moveForwardAtSub3 = robot.drive.actionBuilder(robot.drive.pose)
         //       .strafeToConstantHeading(new Vector2d(4, -34))
           //     .build();
        //Actions.runBlocking(moveForwardAtSub3);

        robot.delay(.2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        robot.delay(.2);

        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(50, -50))
                .build();
        Actions.runBlocking(park);






    }
}


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

        robot.setV4bPos(robot.getV4B_IN_ROBOT());
        robot.setAxlePos(robot.getAXLE_TO_WALL());
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());//slides above high chamber
        robot.update();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9.2, -32.3))
                .build();
        Actions.runBlocking(moveToSub);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        robot.update();

        robot.delay(.3);//was .8

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());

        robot.update();
        robot.delay(.1);
        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action moveToSample = robot.drive.actionBuilder(robot.drive.pose)
                //SPLINE TO CONSTANT HEADING
                //the tangent is like the angle of approach
                //going straight to the right - 0
                //straight to left - 180
                //straight back - -270
                //straight forward - 90
                //https://docs.google.com/document/d/1X0pDgbtgT15IFovgTdUFh0gcSpguYCod1T1fexQR0Fs/edit?tab=t.0


                .splineToConstantHeading(new Vector2d(12, -45), 270)//move back from sub //was (12, -41)
                .splineToConstantHeading(new Vector2d(32, -45), 0)//sideways //was (32.9, -41)
                .splineToConstantHeading(new Vector2d(32, -18), 90)//forward
                .strafeToLinearHeading(new Vector2d(41, -15.3), Math.toRadians(0))//move to sample
                .splineToConstantHeading(new Vector2d(45, -15.3), 0)
                .strafeToConstantHeading(new Vector2d(46, -58))//move to OZ
                .strafeToConstantHeading(new Vector2d(45, -54))
                .strafeToLinearHeading(new Vector2d(37, -56), Math.toRadians(90))//turn to grab spec
                .splineToConstantHeading(new Vector2d(37, -63),90)
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
                .strafeToLinearHeading(new Vector2d(4, -36), Math.toRadians(-90))//move to sub
                .splineToConstantHeading(new Vector2d(4, -29.5), 90)
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

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER()+80);
        robot.update();

        Action moveToSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(38, -59))
                .strafeToLinearHeading(new Vector2d(6, -33.4), Math.toRadians(-90))
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
                .strafeToConstantHeading(new Vector2d(54, -60), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100 , 100))
                .build();
        Actions.runBlocking(park);






    }
}


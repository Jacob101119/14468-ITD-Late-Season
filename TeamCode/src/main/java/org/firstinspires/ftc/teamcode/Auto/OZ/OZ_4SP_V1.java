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
public final class OZ_4SP_V1 extends LinearOpMode {

    BaseRobot robot;
    //MecanumDrive drive;
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

        Action push2 = robot.drive.actionBuilder(robot.drive.pose)
                //SPLINE TO CONSTANT HEADING
                //the tangent is like the angle of approach
                //going straight to the right - 0
                //straight to left - 180
                //straight back - -270
                //straight forward - 90


                .splineToConstantHeading(new Vector2d(12, -45), 270)//move back from sub
                .splineToConstantHeading(new Vector2d(32, -45), 0)//sideways
                .splineToConstantHeading(new Vector2d(32, -18), 90)//forward
                .splineToConstantHeading(new Vector2d(45, -15.3), 0)//move to sample 1//to the right

                .splineToConstantHeading(new Vector2d(45, -56.5), 90)//move to OZ
                .splineToConstantHeading(new Vector2d(45,-15.3),270)//move back
                .splineToConstantHeading(new Vector2d(49,-15.3),0)//move to second sample //to the right
                .splineToConstantHeading(new Vector2d(49, -63), 90)//move to OZ and grab spec //forward
                        .build();

        Actions.runBlocking(push2);



        robot.delay(.2);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        robot.update();

        robot.delay(.2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        robot.update();

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(41, -55), 270)//move back
                .strafeToSplineHeading(new Vector2d(4, -36), Math.toRadians(-90))//move to sub
                .splineToConstantHeading(new Vector2d(4, -29.5), 90)//run into chamber
                .build();
        //Actions.runBlocking(moveToSub2);

        Action moveToSub2NEW = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(41, -55), 270)//move back
                .splineToConstantHeading(new Vector2d(4, -38), 0)//to the right towards sub
                .splineTo(new Vector2d(4, -38), Math.toRadians(-90))//turn 180
                .splineToConstantHeading(new Vector2d(4, -29.5), 90)//run into chamber
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

                .strafeToSplineHeading(new Vector2d(38, -59), Math.toRadians(90))
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
                .splineToConstantHeading(new Vector2d(38, -59),90)
                .strafeToSplineHeading(new Vector2d(6, -33.4), Math.toRadians(-90))
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

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER()+80);
        robot.update();

        Action moveToSub4 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(38, -59), 90)
                .strafeToSplineHeading(new Vector2d(6, -33.4), Math.toRadians(-90))
                .build();
        Actions.runBlocking(moveToSub4);

        robot.delay(.2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        robot.delay(.2);

        robot.setOuttakeSlidesPos(0);
        robot.update();

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER()+80);
        robot.update();

        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(54, -60), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100 , 100))
                .build();
        Actions.runBlocking(park);






    }
}


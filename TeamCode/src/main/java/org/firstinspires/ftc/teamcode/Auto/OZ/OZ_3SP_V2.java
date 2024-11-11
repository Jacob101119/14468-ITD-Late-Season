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
public final class OZ_3SP_V2 extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        robot = new BaseRobot(hardwareMap, new Pose2d(12, -62.91, Math.toRadians(-90)));



        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());



        //test heading reset
        robot.drive.resetHeading();

        waitForStart();



        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());//slides above high chamber
        //robot.update();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -34))
                .build();
        Actions.runBlocking(moveToSub);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        //robot.update();

        robot.delay(1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        //robot.update();
        robot.delay(1);

        Action moveToSample = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -41))//back from sub
                .strafeToConstantHeading(new Vector2d(36, -41))//sideways
                .strafeToConstantHeading(new Vector2d(35, -18))//forward
                .strafeToSplineHeading(new Vector2d(48.5, -18), Math.toRadians(90))//move to sample
                .strafeToConstantHeading(new Vector2d(48.5, -62))
                .build();
        Actions.runBlocking(moveToSample);

        robot.delay(1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        //robot.update();

        robot.delay(1);

        robot.setOuttakeSlidesPos(100);
        //robot.update();

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(48.5, -55))//move back
                .strafeToLinearHeading(new Vector2d(4, -38), Math.toRadians(-90))//move to sub
                .build();
        Actions.runBlocking(moveToSub2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        //robot.update();

        robot.delay(.5);

        Action moveForwardAtSub2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(4, -34))
                .build();
        Actions.runBlocking(moveForwardAtSub2);

        robot.delay(.2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        //robot.update();
        robot.delay(1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        //robot.update();

        robot.delay(1);

        robot.setOuttakeSlidesPos(0);
        //robot.update();


        Action moveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(12, -48))

                .strafeToLinearHeading(new Vector2d(52.5, -59), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(52.5, -62))
                .build();
        Actions.runBlocking(moveToOZ);
        robot.delay(1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        //robot.update();

        robot.delay(1);

        robot.setOuttakeSlidesPos(100);
        //robot.update();

        Action moveToSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(52.5, -59))
                .strafeToLinearHeading(new Vector2d(4, -38), Math.toRadians(-90))
                .build();
        Actions.runBlocking(moveToSub3);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        //robot.update();

        robot.delay(.5);

        Action moveForwardAtSub3 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(4, -34))
                .build();
        Actions.runBlocking(moveForwardAtSub3);

        robot.delay(.2);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ON_HIGH_CHAMBER());
        //robot.update();
        robot.delay(1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        //robot.update();

        robot.delay(1);

        robot.setOuttakeSlidesPos(0);
        //robot.update();

        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(50, -50))
                .build();
        Actions.runBlocking(park);






    }
}


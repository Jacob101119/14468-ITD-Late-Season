package org.firstinspires.ftc.teamcode.Auto.HB;


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

import org.firstinspires.ftc.teamcode.BaseRobot.IntakeAction;




@Autonomous
public final class HB_2S extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        double startY = -62.7;
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, -62.7, Math.toRadians(0)));



        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());



        //test heading reset
        robot.drive.resetHeading();

        waitForStart();

        robot.setAxlePos(robot.getAXLE_TO_WALL());
        robot.setOuttakeSlidesPos(2104);
        robot.update();

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-54, -54), Math.toRadians(45))
                        .strafeToConstantHeading(new Vector2d(-59, -59))
                                .build();
        Actions.runBlocking(moveForwardAtHB1);
        robot.delay(2);
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        Action moveBackFromHB1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-50, -50))

                .build();
        Actions.runBlocking(moveBackFromHB1);



        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action touchLowBar = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-30, -10.6), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-26.7, -10.6),Math.toRadians(0))
                .build();
        Actions.runBlocking(touchLowBar);
        robot.setV4bPos(robot.getV4B_HOVER_OVER_GROUND());
        robot.update();
        robot.delay(.5);
        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_OPEN());
        robot.update();
        robot.delay(6);


    }
}


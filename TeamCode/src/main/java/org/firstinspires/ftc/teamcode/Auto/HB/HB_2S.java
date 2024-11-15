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
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, startY, Math.toRadians(180)));



        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());



        //test heading reset
        robot.drive.resetHeading();

        waitForStart();

        robot.setAxlePos(robot.getAXLE_HB());
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TO_HB());
        robot.update();

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToConstantHeading(new Vector2d(-50, startY))
                                .build();
        Actions.runBlocking(moveForwardAtHB1);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        Action moveBackFromHB1 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-46, startY))
                .build();
        Actions.runBlocking(moveBackFromHB1);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TRANSFER());
        robot.setV4bPos(robot.getV4B_HOVER_OVER_GROUND());
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();


        Action moveToYellow1 = robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(49.5, -42.5), Math.toRadians(90))
                                .build();
        Actions.runBlocking(moveToYellow1);

        robot.setV4bPos(robot.getV4B_INTAKE_POS());
        robot.update();
        robot.delay(.5);

        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_CLOSED());
        robot.update();
        robot.delay(.1);

        Actions.runBlocking(new IntakeAction(robot));//intake action
        robot.delay(.1);

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TO_HB());
        robot.update();

        Action moveToHB2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-55.2, -55.2), Math.toRadians(225))
                .build();
        Actions.runBlocking(moveToHB2);

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.update();

        Action moveBackFromHB2 = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(-50, -50))
                .build();
        Actions.runBlocking(moveBackFromHB2);

        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action touchLowBar = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-26.7, -10.6),Math.toRadians(180))
                .build();
        Actions.runBlocking(touchLowBar);

    }
}


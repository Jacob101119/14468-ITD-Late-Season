package org.firstinspires.ftc.teamcode.Auto.HB;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.BaseRobot.TransferAction;



@Autonomous
public final class HB_4S extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        double startY = -62.7;
        robot = new BaseRobot(hardwareMap, new Pose2d(-39, -62.7, Math.toRadians(0)));




        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);


        //test heading reset
        robot.drive.resetHeading();

        waitForStart();

        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.MAX);
        robot.update();

        Action moveForwardAtHB1 = robot.drive.actionBuilder(robot.drive.pose)

                .splineToConstantHeading(new Vector2d(-59, -59), 270)
                .build();
        Actions.runBlocking(moveForwardAtHB1);
        robot.delay(2);


        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();
        robot.delay(.2);

        Action moveBackFromHB1 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(-50, -50), 90)

                .build();
        Actions.runBlocking(moveBackFromHB1);



        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.transfer);
        robot.setAxlePos(Constants.outtakeAxleConstants.down);
        robot.setV4bPos(Constants.v4bConstants.hover);
        robot.update();

        Action moveTo1stYellow = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToLinearHeading(new Vector2d(-48.5, -39.8), Math.toRadians(270))
                .build();
        Actions.runBlocking(moveTo1stYellow);

        robot.setV4bPos(Constants.v4bConstants.ground);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);
        robot.update();

        robot.delay(.4);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
        robot.delay(.1);

        Actions.runBlocking(new TransferAction(robot));
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.MAX);
        robot.update();

       /* Action moveToHB = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(1, 5)10)
                .build();

        */


        Action touchLowBar = robot.drive.actionBuilder(robot.drive.pose)

                .strafeToLinearHeading(new Vector2d(-30, -10.6), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-26.7, -10.6),Math.toRadians(0))
                .build();
        Actions.runBlocking(touchLowBar);
        robot.setV4bPos(Constants.v4bConstants.hover);
        robot.update();
        robot.delay(.5);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);
        robot.update();
        robot.delay(6);


    }
}


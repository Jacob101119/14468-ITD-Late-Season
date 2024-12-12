package org.firstinspires.ftc.teamcode.Auto.OZ;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BaseRobot.TransferAction;
import org.firstinspires.ftc.teamcode.util.Constants;

import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;


@Autonomous
public final class DONT_USE_OZ_5SP_V2 extends LinearOpMode {

    BaseRobot robot;
    MecanumDrive drive;
    @Override


    public void runOpMode() throws InterruptedException{

        robot = new BaseRobot(hardwareMap, new Pose2d(12, -62.91, Math.toRadians(-90)));







        robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);

        //test heading reset
        robot.drive.resetHeading();

        waitForStart();


        robot.setV4bPos(Constants.v4bConstants.up);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringBelowChamber);//slides right below high chamber for upside down scoring
        robot.update();

        Action moveToSub = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(9, -39.4))
                .afterTime(0, (t) -> {
                    robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringOnChamber);
                    robot.update();
                    return false;})
                .build();
        Actions.runBlocking(moveToSub);
        robot.setIntakeSlidesPos(Constants.intakeSlideConstants.transfer);




        robot.delay(.3);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();

        robot.delay(.1);

        robot.setOuttakeSlidesPos(0);
        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);//axle back into robot for grabbing spec

        robot.setV4bPos(Constants.v4bConstants.hover);
        robot.update();

        double graspingDelay = .4;
        Action transfer3 = robot.drive.actionBuilder(robot.drive.pose)

                //going straight to the right - 0
                //straight to left - 180
                //straight back - -270
                //straight forward - 90
                .strafeToLinearHeading(new Vector2d(45, -45), Math.toRadians(90))//move to sample 1
                .afterTime(0, (t) -> {
                    robot.setAxlePos(Constants.outtakeAxleConstants.down);
                    robot.delay(graspingDelay);
                    robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
                    Actions.runBlocking(new TransferAction(robot));
                    robot.setV4bPos(Constants.v4bConstants.hover);
                    robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
                    robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
                    return false;})

                .splineToConstantHeading(new Vector2d(56, -45), 0)//move to sample 1

                .afterTime(0, (t) -> {
                    robot.setAxlePos(Constants.outtakeAxleConstants.down);
                    robot.delay(graspingDelay);
                    robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
                    Actions.runBlocking(new TransferAction(robot));
                    robot.setV4bPos(Constants.v4bConstants.hover);
                    robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
                    robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
                    return false;})

                .splineToConstantHeading(new Vector2d(62, -45), 0)//move to sample 1

                .afterTime(0, (t) -> {
                    robot.setAxlePos(Constants.outtakeAxleConstants.down);
                    robot.delay(graspingDelay);
                    robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
                    Actions.runBlocking(new TransferAction(robot));
                    robot.setV4bPos(Constants.v4bConstants.hover);
                    robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
                    robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
                    return false;})

                .splineToConstantHeading(new Vector2d(62, -35), 90)//foward
                .splineTo(new Vector2d(62, -35), Math.toRadians(180))//turn
                .splineToConstantHeading(new Vector2d(62, -64), 270)//grab

                .build();
        Actions.runBlocking(transfer3);




        robot.delay(.3);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        robot.update();

        robot.delay(.2);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringBelowChamber);
        robot.update();

        Action moveToSub2 = robot.drive.actionBuilder(robot.drive.pose)
                //.splineToConstantHeading(new Vector2d(41, -55), 270)//move back
                .splineToConstantHeading(new Vector2d(8, -36.6), 0)//move to sub

                //.splineTo(new Vector2d(4, -37), 270)//move back

                .build();
        Actions.runBlocking(moveToSub2);

        //robot.delay(.2);
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringOnChamber);
        robot.update();


        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();



        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
        robot.setOuttakeSlidesPos(0);
        robot.update();


        Action moveToOZ = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(4, -48),90)
                .splineToConstantHeading(new Vector2d(38, -59), 0)
                .splineToConstantHeading(new Vector2d(38, -64.5), 90)

                .build();
        Actions.runBlocking(moveToOZ);
        robot.delay(.2);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        robot.update();

        robot.delay(.2);


        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringBelowChamber);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.update();

        Action moveToSub3 = robot.drive.actionBuilder(robot.drive.pose)
                //.splineToConstantHeading(new Vector2d(38, -59),90)
                .splineToConstantHeading(new Vector2d(6, -38.5), 270)
                //.splineToConstantHeading(new Vector2d(6, -36), 90)//run into chamber
                .build();
        Actions.runBlocking(moveToSub3);
        robot.delay(.2);

        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringOnChamber);
        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();

        robot.delay(.2);
        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);

        robot.setOuttakeSlidesPos(0);
        robot.update();

        Action moveToOZ2 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(4, -48),90)
                .splineToConstantHeading(new Vector2d(38, -59), 0)
                .splineToConstantHeading(new Vector2d(38, -64.5), 90)
                .build();
        Actions.runBlocking(moveToOZ2);


        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        robot.update();

        robot.delay(.2);


        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringBelowChamber);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.update();


        Action moveToSub4 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(38, -59), 270)
                .splineToConstantHeading(new Vector2d(6, -33.4), 270)
                .build();
        Actions.runBlocking(moveToSub4);


        robot.delay(.2);

        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringOnChamber);
        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();

        robot.delay(.2);

        robot.setOuttakeSlidesPos(0);
        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
        robot.update();

        Action moveToOZ3 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(4, -48),90)
                .splineToConstantHeading(new Vector2d(38, -59), 0)
                .splineToConstantHeading(new Vector2d(38, -64.5), 90)
                .build();
        Actions.runBlocking(moveToOZ3);


        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        robot.update();

        robot.delay(.2);


        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringBelowChamber);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.update();

        Action moveToSub5 = robot.drive.actionBuilder(robot.drive.pose)
                .splineToConstantHeading(new Vector2d(38, -59), 270)
                .splineToConstantHeading(new Vector2d(6, -33.4), 270)
                .build();
        Actions.runBlocking(moveToSub5);


        robot.delay(.2);

        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.passThroughScoringOnChamber);
        robot.update();
        robot.delay(.5);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();

        robot.delay(.2);

        robot.setOuttakeSlidesPos(0);
        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);
        robot.update();


        Action park = robot.drive.actionBuilder(robot.drive.pose)
                .strafeToConstantHeading(new Vector2d(54, -60), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100 , 100))
                .build();
        Actions.runBlocking(park);

        robot.setOuttakeSlidesPos(0);
        robot.setV4bPos(Constants.v4bConstants.hover);//ready for intaking from sub
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);
        robot.setGimbalPos(Constants.intakeClawConstants.gimbalReset);




    }
}


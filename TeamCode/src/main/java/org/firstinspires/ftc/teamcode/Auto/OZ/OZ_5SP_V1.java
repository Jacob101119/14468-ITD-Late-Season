package org.firstinspires.ftc.teamcode.Auto.OZ;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;

import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;

import java.util.Vector;


@Autonomous
public final class OZ_5SP_V1 extends LinearOpMode {

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





        robot.delay(.3);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.update();

        robot.delay(.1);

        robot.setOuttakeSlidesPos(0);
        robot.setAxlePos(Constants.outtakeAxleConstants.passThrough);//axle back into robot for grabbing spec
        robot.update();

        Action push3 = robot.drive.actionBuilder(robot.drive.pose)
                //the tangent is like the angle of approach
                //going straight to the right - 0
                //straight to left - 180
                //straight back - -270
                //straight forward - 90

                //super speed - , new TranslationalVelConstraint(90), new ProfileAccelConstraint(-90 , 90)

                //back from sub to first sample
                //.splineToConstantHeading(new Vector2d(12, -44), 90)//move back from sub
                .splineToConstantHeading(new Vector2d(33, -39.4), 180)//sideways
                .splineToConstantHeading(new Vector2d(33, -19.5), 90)//forward
                .splineToConstantHeading(new Vector2d(45, -16), 180)//move to sample 1

                //push
                .splineToConstantHeading(new Vector2d(45, -44), 90)//move to OZ

                //move to 2nd sample
                .splineToConstantHeading(new Vector2d(45,-19.5),270)//move back
                .splineToConstantHeading(new Vector2d(56,-19.5),180)//move to second sample

                //push
                .splineToConstantHeading(new Vector2d(49, -47.5), 90)//move to OZ

                //move to third sample
                .splineToConstantHeading(new Vector2d(55, -18), 270)//back away from oz
                .splineToConstantHeading(new Vector2d(62, -18), 180)//move to sample
                .splineToConstantHeading(new Vector2d(55, -50), 90)//push to OZ and grab spec
                .splineToConstantHeading(new Vector2d(44, -64), 90)// grab spec
                //.splineToConstantHeading(new Vector2d(50, -64), 90)// grab spec
                .build();

        Actions.runBlocking(push3);



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


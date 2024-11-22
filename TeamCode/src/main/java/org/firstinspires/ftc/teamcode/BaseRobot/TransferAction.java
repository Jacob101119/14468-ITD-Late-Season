package org.firstinspires.ftc.teamcode.BaseRobot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;

public class TransferAction implements Action {

    BaseRobot robot;
    ElapsedTime time = new ElapsedTime();



    public TransferAction(BaseRobot b){
        this.robot=b;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {



        //prepare outtake for grabbing sample
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.transfer);//slides up a bit
        robot.setAxlePos(Constants.outtakeAxleConstants.HBScoring);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        //end preparing outtake

        robot.setIntakeSlidesPos(0);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);//close grasper
        robot.setTrayPos(Constants.trayConstants.open);//tray open to receive sample
        robot.update();

        while(time.milliseconds()< 100){

        }
        robot.setGimbalPos(Constants.intakeClawConstants.gimbalReset);//gimbal straight
        robot.update();
        time.reset();

        while(time.milliseconds() < 300){


        }
        robot.setV4bPos(Constants.v4bConstants.tray);//v4b in robot
        robot.setIntakeSlidesPos(0);//slides in
        robot.update();
        time.reset();

        while(time.milliseconds() < 2000){

        }

        robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);//drop sample into tray

            robot.update();
        time.reset();

//        //sample is in tray, now needs to move to outtake
//
//        while (time.milliseconds() < 1000){
//
//        }
//
//
//        robot.setTrayPos(Constants.trayConstants.halfClosed);
//        robot.update();
//        //v4b out of way of the outtake
//        time.reset();
//        while(time.milliseconds()<300){
//
//        }
//        //robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);
//        robot.update();
//        time.reset();

        while(time.milliseconds()<1000){

        }
        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.setAxlePos(Constants.outtakeAxleConstants.down);
        robot.setTrayPos(Constants.trayConstants.closed);//move tray servo

        robot.update();
        time.reset();

        while (time.milliseconds() < 500){//wait 1 second

        }
        robot.setTrayPos(Constants.trayConstants.closed);



        robot.update();
        time.reset();

        while (time.milliseconds() < 500){

        }
        robot.setTrayPos(Constants.trayConstants.open);
        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);//close grasper
        robot.update();
        time.reset();
        while (time.milliseconds() < 400){

        }
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.aboveChamber);
        //robot.setAxlePos(robot.getAXLE_HB());
        robot.update();







        
        return false;
    }



}

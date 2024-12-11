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
        robot.setOuttakeSlidesPos(0);//slides up a bit
        robot.setAxlePos(Constants.outtakeAxleConstants.straightUp);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.looseGrab);

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        //end preparing outtake

        robot.setIntakeSlidesPos(Constants.intakeSlideConstants.transfer);
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.closed);//close grasper
        robot.setGimbalPos(Constants.intakeClawConstants.gimbalReset);//gimbal straight
        robot.update();

        while(time.milliseconds() < 100){


        }
        robot.setV4bPos(Constants.v4bConstants.transfer);//v4b in robot

        robot.update();
        time.reset();

        while(time.milliseconds() < 1000){

        }
        robot.setAxlePos(Constants.outtakeAxleConstants.transfer);

        time.reset();
        while(time.milliseconds() < 1000){

        }

        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        robot.update();
        time.reset();

        while(time.milliseconds() < 1000){

        }
        robot.setIntakeGrasperPos(Constants.intakeClawConstants.open);
        robot.update();
        time.reset();


        while(time.milliseconds()<1000){

        }

        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.update();









        
        return false;
    }



}

package org.firstinspires.ftc.teamcode.BaseRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeAction implements Action {

    BaseRobot robot;
    ElapsedTime time = new ElapsedTime();



    public IntakeAction(BaseRobot b){
        this.robot=b;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {



        //prepare outtake for grabbing sample
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TRANSFER()+400);//slides up a bit
        robot.setAxlePos(robot.getAXLE_HB());

        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        //end preparing outtake

        robot.setIntakeSlidesPos(0);
        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_CLOSED());//close grasper
        robot.setTrayPos(robot.getTRAY_OPEN());//tray open to recieve sample
        robot.update();
        while(time.milliseconds()< 100){

        }
        robot.setGimbalPos(robot.getGIMBAL_RESTING_POS());//gimbal straight
        robot.update();
        time.reset();

        while(time.milliseconds() < 300){


        }
        robot.setV4bPos(robot.getV4B_IN_ROBOT());//v4b in robot
        robot.setIntakeSlidesPos(0);//slides in
        robot.update();
        time.reset();

        while(time.milliseconds() < 2000){

        }

        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_OPEN());//drop sample into tray

            robot.update();
        time.reset();

        //sample is in tray, now needs to move to outtake

        while (time.milliseconds() < 1000){

        }

        robot.setV4bPos(robot.getV4B_UP());
        robot.setTrayPos(robot.getTRAY_HALF_CLOSED());
        robot.update();
        //v4b out of way of the outtake
        time.reset();
        while(time.milliseconds()<300){

        }
        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_CLOSED());
        robot.update();
        time.reset();

        while(time.milliseconds()<1000){

        }
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.setAxlePos(robot.getAXLE_DOWN());
        robot.setTrayPos(.62);//move tray servo

        robot.update();
        time.reset();

        while (time.milliseconds() < 500){//wait 1 second

        }
        robot.setTrayPos(robot.getTRAY_CLOSED());
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TRANSFER());


        robot.update();
        time.reset();

        while (time.milliseconds() < 500){

        }
        robot.setTrayPos(robot.getTRAY_OPEN());
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());//close grasper
        robot.update();
        time.reset();
        while (time.milliseconds() < 400){

        }
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        robot.setAxlePos(robot.getAXLE_HB());
        robot.update();







        
        return false;
    }



}

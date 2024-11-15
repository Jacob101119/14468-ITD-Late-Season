package org.firstinspires.ftc.teamcode.BaseRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class tryTwoIntake implements Action {

    BaseRobot robot;
    ElapsedTime time = new ElapsedTime();



    public tryTwoIntake(BaseRobot b){
        this.robot=b;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {



        //prepare outtake for grabbing sample

        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_TRANSFER());
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
        robot.setTrayPos(robot.getTRAY_CLOSED());
        robot.setAxlePos(robot.getAXLE_DOWN());
        while(time.milliseconds()<1000){

        }
        robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());
        time.reset();


        while(time.milliseconds()<400){

        }
        robot.setOuttakeSlidesPos(robot.getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER());
        robot.setAxlePos(robot.getAXLE_TO_WALL());
        robot.update();







        return false;
    }



}

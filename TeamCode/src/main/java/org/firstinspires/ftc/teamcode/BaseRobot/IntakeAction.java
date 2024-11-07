package org.firstinspires.ftc.teamcode.BaseRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IntakeAction implements Action {

    BaseRobot robot;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        //run stuff here
        robot = new BaseRobot(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        //prepare outtake for grabbing sample
        robot.setAxlePos(robot.getAXLE_TO_TRAY());
        robot.setOuttakeWristPos(robot.getWRIST_TO_TRAY());
        robot.setOuttakeSlidesPos(0);
        //end preparing outtake


        robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_CLOSED());//close grasper
        robot.setV4bPos(robot.getV4B_IN_ROBOT());//v4b in robot
        robot.setIntakeSlidesPos(0);//slides in
        robot.update();

        if (time.milliseconds() > 1000 && robot.leftIntakeSlider.getCurrentPosition() == 0) {//wait 1 second and wait for slides to go in
            robot.setIntakeGrasperPos(robot.getINTAKE_GRASPER_OPEN());
            robot.update();
        }
        time.reset();
        //sample is in tray, now needs to move to outtake

        if (time.milliseconds() > 400){ //move outtake to tray
            robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_OPEN());
            robot.setAxlePos(robot.getAXLE_TO_TRAY());
            robot.setOuttakeWristPos(robot.getWRIST_TO_TRAY());
            robot.update();
        }
        time.reset();

        if (time.milliseconds() > 1000){//wait 1 second
            robot.setOuttakeGrasperPos(robot.getOUTTAKE_GRASPER_CLOSED());//close grasper
            robot.update();
        }
        time.reset();






        
        return false;
    }



}

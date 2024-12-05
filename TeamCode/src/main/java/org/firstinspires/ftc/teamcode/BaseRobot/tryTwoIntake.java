package org.firstinspires.ftc.teamcode.BaseRobot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;

public class tryTwoIntake implements Action {

    BaseRobot robot;
    ElapsedTime time = new ElapsedTime();



    public tryTwoIntake(BaseRobot b){
        this.robot=b;

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {



        //prepare outtake for grabbing sample

        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.transfer);
        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.open);
        robot.setTrayPos(Constants.trayConstants.closed);
        robot.setAxlePos(Constants.outtakeAxleConstants.down);
        while(time.milliseconds()<1000){

        }
        robot.setOuttakeGrasperPos(Constants.outtakeClawConstants.closed);
        time.reset();


        while(time.milliseconds()<400){

        }
        robot.setOuttakeSlidesPos(Constants.outtakeSlideConstants.aboveChamber);
        robot.setAxlePos(Constants.outtakeAxleConstants.specScoring);
        robot.update();







        return false;
    }



}

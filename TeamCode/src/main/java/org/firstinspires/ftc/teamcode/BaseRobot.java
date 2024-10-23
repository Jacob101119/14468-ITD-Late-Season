package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class BaseRobot{

    // Arm Position fields
    private int leftIntakeSliderPos = 0;

    private int rightIntakeSliderPos = 0;
    private int leftOuttakeSliderPos = 0;
    private int rightOuttakeSliderPos = 0;
    public MecanumDrive drive;

    DcMotor leftIntakeSlider;//from the perspective of the robot
    DcMotor rightIntakeSlider;//from the perspective of the robot

    DcMotor leftOuttakeSlider;//from the perspective of the robot
    DcMotor rightOuttakeSlider;//from the perspective of the robot

    Servo intakeGrasper;
    Servo outtakeGrasper;


    public BaseRobot(HardwareMap hwMap){
        this(hwMap, new Pose2d(0,0,0));
    }

    public BaseRobot(HardwareMap hwMap, Pose2d pose){

        drive = new MecanumDrive(hwMap, pose);


        leftIntakeSlider = hwMap.dcMotor.get("leftIntakeSlider");
        rightIntakeSlider = hwMap.dcMotor.get("rightIntakeSlider");
        //rightIntakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to run with encoders and grab current Position
        leftIntakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntakeSliderPos = leftIntakeSlider.getCurrentPosition();

        rightIntakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeSliderPos = rightIntakeSlider.getCurrentPosition();


        //outtake
        leftOuttakeSlider = hwMap.dcMotor.get("leftOuttakeSlider");
        rightOuttakeSlider = hwMap.dcMotor.get("rightOuttakeSlider");
        //rightOuttakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to run with encoders and grab current Position
        leftOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftOuttakeSliderPos = leftOuttakeSlider.getCurrentPosition();

        rightOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightOuttakeSliderPos = rightOuttakeSlider.getCurrentPosition();





        intakeGrasper = hwMap.servo.get("IntakeGrasper");
        intakeGrasper.setPosition(0);

        outtakeGrasper = hwMap.servo.get("outtakeGrasper");
        outtakeGrasper.setPosition(0);
        //end servos



    }



    //hang arm pos --------------

    public void updateIntakeSlidesPos(){
        leftIntakeSlider.setTargetPosition(leftIntakeSliderPos);
        rightIntakeSlider.setTargetPosition(rightIntakeSliderPos);

        if(leftIntakeSliderPos != rightIntakeSliderPos){
            leftIntakeSliderPos = rightIntakeSliderPos;
        }

    }
    public void changeIntakeSliderPos(int deltaPos){
        leftIntakeSliderPos += deltaPos;
        rightIntakeSliderPos += deltaPos;
    }
    public void setIntakeSliderPos(int newPos){
        leftIntakeSliderPos = newPos;
        rightIntakeSliderPos = newPos;
    }

    //outtake -------------------------------------------------------------------------
    public void updateOuttakeSlidesPos(){
        leftOuttakeSlider.setTargetPosition(leftOuttakeSliderPos);
        rightOuttakeSlider.setTargetPosition(rightOuttakeSliderPos);

        if(leftOuttakeSliderPos != rightOuttakeSliderPos){
            leftOuttakeSliderPos = rightOuttakeSliderPos;
        }

    }
    public void changeOuttakeSliderPos(int deltaPos){
        leftOuttakeSliderPos += deltaPos;
        rightOuttakeSliderPos += deltaPos;
    }
    public void setOuttakeSliderPos(int newPos){
        leftOuttakeSliderPos = newPos;
        rightOuttakeSliderPos = newPos;
    }



    public void delay(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds){

        }

    }


}


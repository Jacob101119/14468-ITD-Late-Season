package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



public class BaseRobot{


    //motor powers
    private double INTAKE_SLIDES_POWER = 0.9;
    private double OUTTAKE_SLIDES_POWER = 0.9;

    //end motor powers

    // Arm Position fields
    private int intakeSlidesPos = 0;

    private int outtakeSlidesPos = 0;

    private double gimbalPos = 0;
    private double intakeGrasperPos = 0;
    private double outtakeGrasperPos = 0;
    private double outtakeWristPos = 0;
    private double outtakeAxlePos = 0;
    private double v4bPos = 0;

    //servo constants
    private double V4B_IN_ROBOT = 0;//change
    private double V4B_INTAKE_POS = 0;//change
    private double GIMBAL_RESTING_POS = 0; //change
    private double INTAKE_GRASPER_OPEN = 0;//change
    private double INTAKE_GRASPER_CLOSED = 0;//change
    private double OUTTAKE_GRASPER_CLOSED = 0;//change
    private double OUTTAKE_GRASPER_OPEN = 0;//change
    private double AXLE_TO_WALL = 0;//change
    private double OUTTAKE_WRIST_TO_WALL = 0;//change
    private double AXLE_STRAIGHT_OUT = 0;//change
    private double WRIST_SCORING = 0;//change
    private double AXLE_TO_TRAY = 0;//change
    private double WRIST_TO_TRAY = 0;//change


    //end servo constants

    //motor constants
    private int INTAKE_SLIDES_MAX = 0;//chnage
    private int OUTTAKE_SLIDES_MAX = 0;//change
    private int OUTTAKE_SLIDES_TO_HB = 0;//change
    private int OUTTAKE_SLIDES_SPECIMEN_SCORING = 0;//change

    //end motor constants




    public MecanumDrive drive;

    DcMotor leftIntakeSlider;//from the perspective of the robot
    DcMotor rightIntakeSlider;//from the perspective of the robot

    DcMotor leftOuttakeSlider;//from the perspective of the robot
    DcMotor rightOuttakeSlider;//from the perspective of the robot

    Servo intakeGrasper;
    Servo outtakeGrasper;
    Servo outtakeWrist;
    Servo v4b;
    Servo intakeGimbal;
    Servo outtakeAxle;



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

        rightIntakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeSlidesPos = leftIntakeSlider.getCurrentPosition();


        //outtake
        leftOuttakeSlider = hwMap.dcMotor.get("leftOuttakeSlider");
        rightOuttakeSlider = hwMap.dcMotor.get("rightOuttakeSlider");
        //rightOuttakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to run with encoders and grab current Position
        leftOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlidesPos = leftOuttakeSlider.getCurrentPosition();

        rightOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);






        intakeGrasper = hwMap.servo.get("IntakeGrasper");
        intakeGrasper.setPosition(0);
        intakeGrasperPos = intakeGrasper.getPosition();

        outtakeGrasper = hwMap.servo.get("outtakeGrasper");
        outtakeGrasper.setPosition(0);
        outtakeGrasperPos = outtakeGrasper.getPosition();

        outtakeWrist = hwMap.servo.get("outtakeWrist");
        outtakeWrist.setPosition(0);
        outtakeWristPos = outtakeWrist.getPosition();

        v4b = hwMap.servo.get("v4b");
        v4b.setPosition(0);
        v4bPos = v4b.getPosition();

        intakeGimbal = hwMap.servo.get("intakeGimbal");
        intakeGimbal.setPosition(0);
        gimbalPos = intakeGimbal.getPosition();

        outtakeAxle = hwMap.servo.get("outtakeAxle");
        outtakeAxle.setPosition(0);
        outtakeAxlePos = outtakeAxle.getPosition();
        //end servos



    }

    public void resetAll(){
        setIntakeSlidesPos(0);
        setOuttakeSliderPos(0);
        v4b.setPosition(V4B_IN_ROBOT);
        outtakeAxle.setPosition(AXLE_TO_TRAY);
        outtakeWrist.setPosition(WRIST_TO_TRAY);

    }
    public void intakingFromGround(){
        setOuttakeSliderPos(0);
        setIntakeSlidesPos(0);
        v4b.setPosition(V4B_INTAKE_POS);
        intakeGrasper.setPosition(INTAKE_GRASPER_OPEN);
        outtakeAxle.setPosition(AXLE_TO_TRAY);
        outtakeWrist.setPosition(WRIST_TO_TRAY);
    }

    public void SpecimenScoring(){
        setIntakeSlidesPos(0);
        setOuttakeSliderPos(OUTTAKE_SLIDES_SPECIMEN_SCORING);
        outtakeAxle.setPosition(AXLE_STRAIGHT_OUT);
        outtakeWrist.setPosition(WRIST_SCORING);
        outtakeGrasper.setPosition(OUTTAKE_GRASPER_CLOSED);
    }
    public void HighBucketScoring(){
        setIntakeSlidesPos(0);
        setOuttakeSliderPos(OUTTAKE_SLIDES_TO_HB);
        outtakeWrist.setPosition(WRIST_SCORING);
    }

    public void updateAll(){
        updateGimbalPos();
        updateIntakeSlidesPos();
        updateOuttakeSlidesPos();
    }



    public void updateGimbalPos(){
        intakeGimbal.setPosition(gimbalPos);
    }
    public void changeGimbalPos(double deltaPos){
        gimbalPos += deltaPos;
    }
    public void setGimbalPos(double newPos){
        gimbalPos = newPos;
    }

    public void updateWristPos(){
        outtakeWrist.setPosition(outtakeWristPos);
    }
    public void setOuttakeWristPos(double newPos){
        outtakeWristPos = newPos;
    }
    public void changeWristPos(double deltaPos){
        outtakeWristPos += deltaPos;
    }

    public void updateAxlePos(){
        outtakeAxle.setPosition(outtakeAxlePos);
    }
    public void changeAxlePos(double deltaPos){
        outtakeAxlePos += deltaPos;
    }
    public void setAxlePos(double newPos){
        outtakeAxlePos = newPos;
    }

    public void updateIntakeGrasperPos(){
        intakeGrasper.setPosition(intakeGrasperPos);
    }
    public void changeIntakeGrasperPos(double deltaPos){
        intakeGrasperPos += deltaPos;
    }
    public void setIntakeGrasperPos(double newPos){
        intakeGrasperPos = newPos;
    }

    public void updateOuttakeGrasperPos(){
        outtakeGrasper.setPosition(outtakeGrasperPos);
    }
    public void changeOuttakeGrasperPos(double deltaPos){
        outtakeGrasperPos += deltaPos;
    }
    public void setOuttakeGrasperPos(double newPos){
        outtakeGrasperPos = newPos;
    }

    public void updateV4bPos(){
        v4b.setPosition(v4bPos);
    }
    public void changeV4bPos(double deltaPos){
        v4bPos += deltaPos;
    }
    public void setV4bPos(double newPos){
        v4bPos = newPos;
    }



    public void updateIntakeSlidesPos(){

        rightIntakeSlider.setTargetPosition(intakeSlidesPos);
        leftIntakeSlider.setTargetPosition(intakeSlidesPos);

        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightIntakeSlider.setPower(INTAKE_SLIDES_POWER);
        leftIntakeSlider.setPower(INTAKE_SLIDES_POWER);

        //if(leftIntakeSliderPos != rightIntakeSliderPos){
          //  leftIntakeSliderPos = rightIntakeSliderPos;
        //}


    }
    public void changeIntakeSlidesPos(int deltaPos){
        intakeSlidesPos += deltaPos;

    }
    public void setIntakeSlidesPos(int newPos){
        intakeSlidesPos = newPos;

    }

    //outtake -------------------------------------------------------------------------
    public void updateOuttakeSlidesPos(){
        leftOuttakeSlider.setTargetPosition(outtakeSlidesPos);
        rightOuttakeSlider.setTargetPosition(outtakeSlidesPos);

        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightOuttakeSlider.setPower(INTAKE_SLIDES_POWER);
        leftOuttakeSlider.setPower(INTAKE_SLIDES_POWER);

       // if(leftOuttakeSliderPos != rightOuttakeSliderPos){
         //   leftOuttakeSliderPos = rightOuttakeSliderPos;
        //}

        if(outtakeSlidesPos > OUTTAKE_SLIDES_MAX){
            outtakeSlidesPos = OUTTAKE_SLIDES_MAX;

        }

    }
    public void changeOuttakeSlidesPos(int deltaPos){
        outtakeSlidesPos += deltaPos;
    }
    public void setOuttakeSlidesPos(int newPos){
        outtakeSlidesPos = newPos;

    }



    public void delay(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds){

        }

    }


    //servos
    public double getGimbalPos(){
        return gimbalPos;
    }
    public double getIntakeGrasperPos(){
        return intakeGrasperPos;
    }
    public double getOuttakeGrasperPos(){
        return outtakeGrasperPos;
    }
    public double getOuttakeWristPos(){
        return outtakeWristPos;
    }
    public double getOuttakeAxlePos(){
        return outtakeAxlePos;
    }
    public double getV4bPos(){
        return v4bPos;
    }
    //end servos

    //powers
    public double getINTAKE_SLIDES_POWER(){
        return INTAKE_SLIDES_POWER;
    }
    public double getOUTTAKE_SLIDES_POWER(){
        return OUTTAKE_SLIDES_POWER;
    }
    //end powers

    //servo constants
    public double getV4B_IN_ROBOT(){
        return V4B_IN_ROBOT;
    }
    public double getV4B_INTAKE_POS(){
        return V4B_INTAKE_POS;
    }
    public double getGIMBAL_RESTING_POS(){
        return GIMBAL_RESTING_POS;
    }
    public double getINTAKE_GRASPER_OPEN(){
        return INTAKE_GRASPER_OPEN;
    }
    public double getINTAKE_GRASPER_CLOSED(){
        return INTAKE_GRASPER_CLOSED;
    }
    public double getOUTTAKE_GRASPER_CLOSED(){
        return OUTTAKE_GRASPER_CLOSED;
    }
    public double getOUTTAKE_GRASPER_OPEN(){
        return OUTTAKE_GRASPER_OPEN;
    }
    public double getAXLE_TO_WALL(){
        return AXLE_TO_WALL;
    }
    public double getAXLE_TO_TRAY(){
        return AXLE_TO_TRAY;
    }
    public double getAXLE_STRAIGHT_OUT(){
        return AXLE_STRAIGHT_OUT;
    }
    public double getOUTTAKE_WRIST_TO_WALL(){
        return OUTTAKE_WRIST_TO_WALL;
    }
    public double getWRIST_TO_TRAY(){
        return WRIST_TO_TRAY;
    }
    public double getWRIST_SCORING(){
        return WRIST_SCORING;
    }
    public int getIntakeSlidesPos(){
        return intakeSlidesPos;
    }
    public int getOuttakeSlidesPos(){
        return outtakeSlidesPos;
    }
    public int getINTAKE_SLIDES_MAX(){
        return INTAKE_SLIDES_MAX;
    }
    public int getOUTTAKE_SLIDES_MAX(){
        return OUTTAKE_SLIDES_MAX;
    }
    public int getOUTTAKE_SLIDES_TO_HB(){
        return OUTTAKE_SLIDES_TO_HB;
    }
    public int getOUTTAKE_SLIDES_SPECIMEN_SCORING(){
        return OUTTAKE_SLIDES_SPECIMEN_SCORING;
    }


}


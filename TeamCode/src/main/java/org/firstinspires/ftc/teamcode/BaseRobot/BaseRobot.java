package org.firstinspires.ftc.teamcode.BaseRobot;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;


import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;


public class BaseRobot{



    //color sensor



    //end color sensor


    //motor powers
    private final double INTAKE_SLIDES_POWER = 0.7;
    private final double OUTTAKE_SLIDES_POWER = 0.9;

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
    private double trayPos = 0;

    //servo constants
    private final double V4B_IN_ROBOT = .7638;
    private final double V4B_UP = .45;
    private final double V4B_RESTING_POS = .6544;
    private final double V4B_INTAKE_POS = .1;
    private final double V4B_HOVER_OVER_GROUND = .2397;
    private final double GIMBAL_RESTING_POS = .1862;
    private final double INTAKE_GRASPER_OPEN = .710;
    private final double INTAKE_GRASPER_CLOSED = .875;
    private final double OUTTAKE_GRASPER_CLOSED = .573;
    private final double OUTTAKE_GRASPER_OPEN = .8656;


    private final double WRIST_TO_WALL = 0;//change
    private final double WRIST_STRAIGHT = 0;//change
    private final double WRIST_SCORING = 0;//change

    private final double AXLE_TO_WALL = .1961;
    private final double AXLE_HB = .3949;
    private final double AXLE_DOWN = .9773;
    //private final double AXLE_IN_ROBOT = .5;
    private final double AXLE_PASS_THROUGH = .6799;//change

    private final double WRIST_TO_TRAY = 0;//change
    private final double TRAY_CLOSED = .7;//moves the tray servo to bring the sample in
    private final double TRAY_HALF_CLOSED = .5;
    private final double TRAY_OPEN = .83;

    //end servo constants

    //motor constants
    private int INTAKE_SLIDES_MAX = 1661;
    private final int OUTTAKE_SLIDES_MAX = 3100;
    private final int OUTTAKE_SLIDES_TO_HB = 3070;
    private final int OUTTAKE_SLIDES_TRANSFER = 390;
    private final int OUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER = 1685;
    private final int OUTTAKE_SLIDES_PASS_THROUGH_BELOW_CHAMBER = 1685;
    private final int OUTTAKE_SLIDES_PASS_THROUGH_ON_HIGH_CHAMBER = 2552;
    private final int OUTTAKE_SLIDES_ON_HIGH_CHAMBER = 890;
    private final int OUTTAKE_SLIDES_UPSIDE_DOWN_HIGH_CHAMBER = 1151;

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
    Servo tray;

    public RevColorSensorV3 colorSensor;



    public BaseRobot(HardwareMap hwMap){
        this(hwMap, new Pose2d(0,0,0));
    }

    public BaseRobot(HardwareMap hwMap, Pose2d pose){

        drive = new MecanumDrive(hwMap, pose);

        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");
        colorSensor.enableLed(true);


        leftIntakeSlider = hwMap.dcMotor.get("leftIntakeSlider");
        rightIntakeSlider = hwMap.dcMotor.get("rightIntakeSlider");
        //rightIntakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set to run with encoders and grab current Position
        leftIntakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightIntakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightIntakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeSlidesPos = leftIntakeSlider.getCurrentPosition();


        //outtake
        leftOuttakeSlider = hwMap.dcMotor.get("leftOuttakeSlider");
        leftOuttakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        rightOuttakeSlider = hwMap.dcMotor.get("rightOuttakeSlider");

        leftOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlidesPos = leftOuttakeSlider.getCurrentPosition();

        rightOuttakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        tray = hwMap.servo.get("tray");
        //tray.setPosition(Constants.trayConstants.open);
        trayPos = tray.getPosition();

        intakeGrasper = hwMap.servo.get("intakeGrasper");
        //intakeGrasper.setPosition(Constants.LoonyClawConstants.closed);
        intakeGrasperPos = intakeGrasper.getPosition();

        outtakeGrasper = hwMap.servo.get("outtakeGrasper");
        outtakeGrasper.setPosition(Constants.outtakeClawConstants.closed);
        outtakeGrasperPos = outtakeGrasper.getPosition();

        outtakeWrist = hwMap.servo.get("outtakeWrist");

        outtakeWristPos = outtakeWrist.getPosition();

        v4b = hwMap.servo.get("v4b");
        //v4b.setPosition(Constants.v4bConstants.tray);
        v4bPos = v4b.getPosition();

        intakeGimbal = hwMap.servo.get("intakeGimbal");
        intakeGimbal.setPosition(Constants.intakeClawConstants.gimbalReset);
        gimbalPos = intakeGimbal.getPosition();

        outtakeAxle = hwMap.servo.get("outtakeAxle");
        outtakeAxle.setPosition(Constants.outtakeAxleConstants.specScoring);
        outtakeAxlePos = outtakeAxle.getPosition();
        //end servos



    }

    public String detectColor() {
        // Get the RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (isRedSample(red, green, blue)) {
            return "RedSample";
        } else if (isBlueSample(red, green, blue)) {
            return "BlueSample";
        } else if (isYellowSample(red, green, blue)) {
            return "YellowSample";
        } else {
            return "No Color"; // Default when no color is detected
        }
    }

    private boolean isRedSample(int red, int green, int blue) {
        return red > 100 && green < 80 && blue < 80;
    }
    private boolean isBlueSample(int red, int green, int blue) {
        return blue > 100 && red < 80 && green < 80;//check if all of these are true, if not return false
    }
    private boolean isYellowSample(int red, int green, int blue) {
        return red > 100 && green > 100 && blue < 80;
    }


    //what would cause a camera video t



        public void resetAll(){
        setIntakeSlidesPos(0);
        setOuttakeSlidesPos(0);
        v4b.setPosition(Constants.v4bConstants.tray);
        //outtakeAxle.setPosition();

        }

    public void SpecimenScoring(){
        setOuttakeSlidesPos(Constants.outtakeSlideConstants.aboveChamber);
        outtakeAxle.setPosition(Constants.outtakeAxleConstants.specScoring);
        //outtakeWrist.setPosition(WRIST_SCORING);
        outtakeGrasper.setPosition(Constants.outtakeClawConstants.closed);
    }
    public void HighBucketScoring(){
        setOuttakeSlidesPos(Constants.outtakeSlideConstants.HighBucket);
        setAxlePos(Constants.outtakeAxleConstants.HBScoring);
    }

    public void resetOuttake(){
        setOuttakeSlidesPos(0);
        setAxlePos(Constants.outtakeAxleConstants.specScoring);
    }
    public void resetIntake(){
        setV4bPos(Constants.v4bConstants.farInTrayForPassThrough);
        setIntakeSlidesPos(0);
        setTrayPos(Constants.trayConstants.open);
        setIntakeGrasperPos(Constants.intakeClawConstants.closed);
    }


    public void update(){
        //motors

        //updateIntakeSlidesPos();
        updateOuttakeSlidesPos();

        //servos
        updateAxlePos();
        updateTrayPos();
        updateGimbalPos();
        updateIntakeGrasperPos();
        updateOuttakeGrasperPos();
        updateWristPos();
        updateV4bPos();
    }

    public void TeleopUpdate(){
        //motors


        //servos
        updateAxlePos();
        updateTrayPos();
        updateGimbalPos();
        updateIntakeGrasperPos();
        updateOuttakeGrasperPos();
        updateWristPos();
        updateV4bPos();
    }

    public void intakePos(){
        setV4bPos(Constants.v4bConstants.ground);
        setIntakeGrasperPos(getINTAKE_GRASPER_CLOSED());
        setGimbalPos(Constants.intakeClawConstants.gimbalReset);
        setTrayPos(Constants.trayConstants.open);
    }

    public void servoTestingUpdate(){
        intakeGimbal.setPosition(gimbalPos);
        outtakeAxle.setPosition(outtakeAxlePos);
        tray.setPosition(trayPos);
        intakeGrasper.setPosition(intakeGrasperPos);
        outtakeGrasper.setPosition(outtakeGrasperPos);
        v4b.setPosition(v4bPos);
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
        //if(outtakeSlidesPos < OUTTAKE_SLIDES_TRANSFER && outtakeAxlePos > AXLE_IN_ROBOT){
          //  outtakeAxlePos = AXLE_IN_ROBOT;
        //}
        outtakeAxle.setPosition(outtakeAxlePos);
    }
    public void changeAxlePos(double deltaPos){
        outtakeAxlePos += deltaPos;
    }
    public void setAxlePos(double newPos){
        outtakeAxlePos = newPos;
    }

    public void updateTrayPos(){
        tray.setPosition(trayPos);
    }
    public void changeTrayPos(double deltaPos){
        trayPos += deltaPos;
    }
    public void setTrayPos(double newPos){

        trayPos = newPos;
    }

    public void updateIntakeGrasperPos(){

        //new limits to let v4b move
        //if(intakeGrasperPos != INTAKE_GRASPER_CLOSED && intakeGrasperPos != INTAKE_GRASPER_OPEN){
          //  intakeGrasperPos = INTAKE_GRASPER_CLOSED;
        //}
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

        rightIntakeSlider.setPower(Constants.intakeSlideConstants.power);
        leftIntakeSlider.setPower(Constants.intakeSlideConstants.power);

        if(intakeSlidesPos > Constants.intakeSlideConstants.MAX){
            intakeSlidesPos = Constants.intakeSlideConstants.MAX;//limit
        }
        if(intakeSlidesPos < 0){
            intakeSlidesPos = 0;//limit
        }


    }
    public void changeIntakeSlidesPos(int deltaPos){
        intakeSlidesPos += deltaPos;

    }


    public void setIntakeSlidesPos(int newPos){
        intakeSlidesPos = newPos;

    }

    public void setIntakePower(double power) {
        rightIntakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlidesPos = leftIntakeSlider.getCurrentPosition();
        rightIntakeSlider.setPower(power);
        leftIntakeSlider.setPower(power);
    }

    //outtake -------------------------------------------------------------------------
    public void updateOuttakeSlidesPos(){
        leftOuttakeSlider.setTargetPosition(outtakeSlidesPos);
        rightOuttakeSlider.setTargetPosition(outtakeSlidesPos);

        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightOuttakeSlider.setPower(Constants.outtakeSlideConstants.power);
        leftOuttakeSlider.setPower(Constants.outtakeSlideConstants.power);

       // if(leftOuttakeSliderPos != rightOuttakeSliderPos){
         //   leftOuttakeSliderPos = rightOuttakeSliderPos;
        //}

        if(outtakeSlidesPos > Constants.outtakeSlideConstants.MAX){
            outtakeSlidesPos = Constants.outtakeSlideConstants.MAX;

        }
        if (outtakeSlidesPos < 0){
            outtakeSlidesPos = 0;
        }

    }

    public void changeOuttakeSlidesPos(int deltaPos){
        outtakeSlidesPos += deltaPos;
    }
    public void setOuttakeSlidesPos(int newPos){
        outtakeSlidesPos = newPos;

    }

    public void setOuttakePower(double power) {
        rightOuttakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOuttakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSlidesPos = leftOuttakeSlider.getCurrentPosition();
        rightOuttakeSlider.setPower(power);
        leftOuttakeSlider.setPower(power);
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
    public double getTrayPos(){
        return trayPos;
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
    public double getV4B_UP(){
        return V4B_UP;
    }
    public double getV4B_RESTING_POS(){
        return V4B_RESTING_POS;
    }
    public double getV4B_HOVER_OVER_GROUND(){
        return V4B_HOVER_OVER_GROUND;
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

    public double getAXLE_HB(){
        return AXLE_HB;
    }
    public double getAXLE_DOWN(){
        return AXLE_DOWN;
    }
    public double getAXLE_PASS_THROUGH(){
        return AXLE_PASS_THROUGH;
    }
    public double getOUTTAKE_WRIST_TO_WALL(){
        return WRIST_TO_WALL;
    }
    public double getWRIST_TO_TRAY(){
        return WRIST_TO_TRAY;
    }
    public double getWRIST_SCORING(){
        return WRIST_SCORING;
    }
    public double getWRIST_STRAIGHT(){
        return WRIST_STRAIGHT;
    }
    public double getWRIST_TO_WALL(){
        return WRIST_TO_WALL;
    }
    public double getTRAY_CLOSED(){
        return TRAY_CLOSED;
    }
    public double getTRAY_HALF_CLOSED(){
        return TRAY_HALF_CLOSED;
    }
    public double getTRAY_OPEN(){
        return TRAY_OPEN;
    }


    public int getIntakeSlidesPos(){
        return intakeSlidesPos;
    }
    public int getOuttakeSlidesPos(){
        return outtakeSlidesPos;
    }
    public int getOUTTAKE_SLIDES_TRANSFER(){
        return OUTTAKE_SLIDES_TRANSFER;
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
    public int getOUTTAKE_SLIDES_UPSIDE_DOWN_HIGH_CHAMBER(){
        return OUTTAKE_SLIDES_UPSIDE_DOWN_HIGH_CHAMBER;
    }
    public int getOUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER(){
        return OUTTAKE_SLIDES_ABOVE_HIGH_CHAMBER;
    }
    public int getOUTTAKE_SLIDES_ON_HIGH_CHAMBER(){
        //change

        return OUTTAKE_SLIDES_ON_HIGH_CHAMBER;
    }
    public int getOUTTAKE_SLIDES_PASS_THROUGH_ON_HIGH_CHAMBER(){
        return OUTTAKE_SLIDES_PASS_THROUGH_ON_HIGH_CHAMBER;
    }
    public int getOUTTAKE_SLIDES_PASS_THROUGH_BELOW_CHAMBER(){
        return OUTTAKE_SLIDES_PASS_THROUGH_BELOW_CHAMBER;
    }




}


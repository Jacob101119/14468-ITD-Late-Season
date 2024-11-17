package org.firstinspires.ftc.teamcode.Teleops;

//import com.google.blocks.ftcrobotcontroller.runtime.CRServoAccess;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.BaseRobot.BaseRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class ColorSensorTest extends LinearOpMode {

    private com.qualcomm.robotcore.hardware.HardwareMap HardwareMap;
    BaseRobot robot;







    @Override
    public void runOpMode() throws InterruptedException {



        robot.colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        robot.colorSensor.enableLed(true);

        waitForStart();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        robot = new BaseRobot(hardwareMap);

        while(!isStopRequested() && opModeIsActive()){

            // Read the color sensor values (RGB)
            int red = robot.colorSensor.red();
            int green = robot.colorSensor.green();
            int blue = robot.colorSensor.blue();


            String detectedColor = robot.detectColor();
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();

            // Output the RGB values to the telemetry (Driver Station)
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);



            telemetry.update();
            //end telemetry



            //_____________________________________________________________________________________
        }
    }
}


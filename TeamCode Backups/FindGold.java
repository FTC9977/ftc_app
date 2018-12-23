package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="FindGold", group="test")

public class FindGold extends LinearOpMode {


    private GoldAlignDetector detector;
    //private DcMotor leftFront, leftRear, rightFront, rightRear;


 @Override

 public void runOpMode (){
     telemetry.addData("Status", "Initialized");
     telemetry.update();


     detector = new GoldAlignDetector();
     detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
     detector.useDefaults();
     detector.enable();


     waitForStart();

     if(detector.getAligned() == false) {
          telemetry.addLine("Not Aligned");
          telemetry.update();
     } else if (detector.getAligned() == true) {
         telemetry.addLine("Found GOLD LADDY!");
         telemetry.addData("X Position is: ", detector.getXPosition());
     }
     /* while(detector.getAligned() == false && !isStopRequested()){
          telemetry.addData("Aligned", detector.getAligned());
          telemetry.addData("X Position is: ", detector.getXPosition());
          telemetry.update();
      */
     }


 }





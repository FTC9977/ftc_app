package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="LimitSW", group = "Testing")
// @Disabled


public class LimitSWTest extends LinearOpMode {

    DcMotor lander;
    DigitalChannel digIn;

    @Override

    public void runOpMode() throws InterruptedException{

      digIn = hardwareMap.get(DigitalChannel.class, "digin");
      telemetry.addData("<", "Fully Initialized");

      lander = hardwareMap.dcMotor.get("lander");


      // wait for start button

      waitForStart();

      while (opModeIsActive()) {

          while (!digIn.getState() == false){
              lander.setPower(1);
              telemetry.addData("Is pressed?   ", isPressed());
              telemetry.update();
          }

          lander.setPower(-1);
          telemetry.addData("Current state of pin is ", digIn.getState());

      }


    }

    public boolean isPressed() {
        //Checks if the current state of the input pin is true
        return digIn.getState();
    }

}

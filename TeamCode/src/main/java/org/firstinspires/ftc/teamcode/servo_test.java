package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="Servo_Test", group="S")
public class servo_test extends LinearOpMode {

    Servo s;
   @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.servo.get("s");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a == true) {
                s.setPosition(1);
            } else if (gamepad1.b == true) {
                s.setPosition(0);
            }
        }
    }

}

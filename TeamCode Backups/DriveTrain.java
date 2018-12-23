package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

/**
 * Created by eric on 12/2/18.
 *
 * This code was adopted by referencing code from:
 * https://github.com/ethan-schaffer/Sample-FTC-Code/blob/master/ftc_app-master/ftc_app-master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumSamples/MecanumTeleOp.java
 *
 *
 */

@TeleOp(name="MecanumTeleOP", group="CS")
//@Disabled

// delete  the claw code on here ok!!!!!!!!!!!!!!!!
public class DriveTrain extends OpMode {

    private static final double TRIGGERTHRESHOLD = .2;
    private static final double ACCEPTINPUTTHRESHOLD =  .05;
    private static final double SCALEDPOWER = 1;  // The emphasis is on the current controller reading (vs. current motor power) on the drive train

    private static DcMotor leftFrontWheel, leftBackWheel, rightFrontWheel, rightBackWheel, landerHook, depositArm, pivotArm;
    private static Servo sweep, extenderArm;


  /*   TO DO LIST:
   *
   *   1. Add Servo controls for ingest arm pickup
   *   2. Add motor controls for arm extension/retraction
   *   3. Add motor controls for mineral delivery into lander
   *   4. Add motor controls form Lander Hook
   */


    @Override

    public void init () {

        // Following are for Basic Mecanum Drive in Arcade Mode
        leftFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT1NAME);
        leftBackWheel = hardwareMap.dcMotor.get(UniversalConstants.LEFT2NAME);
        rightFrontWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT1NAME);
        rightBackWheel = hardwareMap.dcMotor.get(UniversalConstants.RIGHT2NAME);
        leftFrontWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        // The following hardware mappings are additional DcMotor appliances
        landerHook = hardwareMap.dcMotor.get(UniversalConstants.LANDERHOOK);
        depositArm = hardwareMap.dcMotor.get(UniversalConstants.DEPOSIT);
        pivotArm = hardwareMap.dcMotor.get(UniversalConstants.PVT_EXTEND);

        // The following hardware mappings are additional Servo appliances
        extenderArm = hardwareMap.servo.get(UniversalConstants.EXTEND);



        //---------------------------------------------------------------- claw code VVVVVVVVVVVVV



    }

    @Override
    public void loop() {
        double inputY = Math.abs(gamepad1.left_stick_y) > ACCEPTINPUTTHRESHOLD ? -gamepad1.left_stick_y : 0;
        double inputX = Math.abs(gamepad1.left_stick_x) > ACCEPTINPUTTHRESHOLD ? gamepad1.left_stick_x : 0;
        double inputC = Math.abs(gamepad1.right_stick_x)> ACCEPTINPUTTHRESHOLD ? gamepad1.right_stick_x: 0;

        arcadeMecanum(inputY, inputX, inputC, leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel);

        /* --------------------------------------------------------------------- OLD claw code VVVVVVVVVVVVVVV
         * Replace with New Rover Ruckus code for:
         *   1. Manipulating the Sweeper Ingest
         *   2. Extending/Retracking the extenderArm Mecahnism
         *   3. Manipulating the Pivot arm for Mineral Delivery
         *   4. Define controls for Gamepad2, which controls this section of code

        motor.setPower(gamepad2.left_stick_y * .5);
        // servo.setPosition(0);
        //servo1.setPosition(0);
        if (gamepad2.x == true){
            servo1.setPosition(0.3);
            servo.setPosition(0.6);
        } else if (gamepad2.a == true){
            servo.setPosition(0.02);
            servo1.setPosition(0.98);
        } else if (gamepad2.right_trigger == 1) {
            servo.setPosition(.3);
            servo1.setPosition(.65);
            // added above code, when robot is at crypto box, driver presses right trigger to cause gripper
            // to only partially open.  Trying to solve the problem where a full open at the crypto box
            // was knocking already scored glpyhs out of the crypto-box and de-scoring them.

        }

     */

      // Insert New Rover Ruckus Code for Mineral Ingest/Delivery options HERE







      // END of Code segment for Mineral Ingest/Delivery Options

    }
    // The following is the code to control the driveTrain in Arcade mode.
    //  Note:  This is for TeleOP Mode only.. Not intended to be used in Autonomous Mode.
    //
    // y - forwards
    // x - side
    // c - rotation
    public static void arcadeMecanum(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }
        double scaledPower = SCALEDPOWER;

        leftFront.setPower(leftFrontVal*scaledPower+leftFront.getPower()*(1-scaledPower));
        rightFront.setPower(rightFrontVal*scaledPower+rightFront.getPower()*(1-scaledPower));
        leftBack.setPower(leftBackVal*scaledPower+leftBack.getPower()*(1-scaledPower));
        rightBack.setPower(rightBackVal*scaledPower+rightBack.getPower()*(1-scaledPower));
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.io.File;
import java.util.Locale;


/**
 * Created by Eric on 11/06/18 - Original Version
 *
 * Purpose:
 *  The purpose of this java.class file is to define all the specific hardware for
 *  the competition Robot - Rover Ruckus 2018
 *
 *  Usage:
 *
 *  Within this file, you should define mappings for:
 *      - DC Motors
 *      - Servo's
 *      - Sensors
 *      - Misc. Mapped Hardware (ICS, etc..)
 *
 *  Note:
 *      You may need to included additional "import com.qualcomm.x.x" statements above
 *      to "pull-in" additional Java Libraries
 *
 */

public class RRHardwareMap {

        public DcMotor DriveLeftFront =null,DriveLeftRear =null
                ,DriveRightFront = null,DriveRightRear = null,Lift = null;



        /* Local OpMode Members */
        HardwareMap hwMap     = null;
        private ElapsedTime period = new ElapsedTime();

        //~~~~ IMU:
        public BNO055IMU imu;
        public boolean calibratedIMU;

        public void init (HardwareMap robot_AS) {

                hwMap = robot_AS;

                // Define and Initialize Motors

                DriveLeftFront = hwMap.dcMotor.get("LF");
                DriveLeftRear = hwMap.dcMotor.get("LR");
                DriveRightFront = hwMap.dcMotor.get("RF");
                DriveRightRear = hwMap.dcMotor.get("RR");

                DriveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                DriveRightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                DriveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                DriveLeftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                DriveLeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
                DriveLeftRear.setDirection(DcMotorSimple.Direction.FORWARD);
                DriveRightRear.setDirection(DcMotorSimple.Direction.REVERSE);
                DriveRightFront.setDirection(DcMotorSimple.Direction.REVERSE);


                // Set all motors Power to Zero Power
                DriveRightFront.setPower(0);
                DriveRightRear.setPower(0);
                DriveLeftFront.setPower(0);
                DriveLeftRear.setPower(0);

                //------------------------------------------------------------
                // IMU - BNO055
                // Set up the parameters with which we will use our IMU.
                // + 9 degrees of freedom
                // + use of calibration file (see calibration program)
                //------------------------------------------------------------
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
                parameters.loggingEnabled = true;
                parameters.loggingTag = "IMU";
                parameters.mode = BNO055IMU.SensorMode.NDOF;

                parameters.accelerationIntegrationAlgorithm = null;

                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hwMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                // Read the IMU configuration from the data file saved during calibration.
                // Using try/catch allows us to be specific about the error instead of
                // just showing a NullPointer exception that could come from anywhere in the program.
                calibratedIMU = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        } catch (Exception e) {
            calibratedIMU = false;
        }
        }
        //----------------------------------------------------------------------------------------------
        // Formatting
        //----------------------------------------------------------------------------------------------

        public String formatAngle(AngleUnit angleUnit, double angle) {
                return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
        }

        public String formatDegrees(double degrees){
                return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
        }


}

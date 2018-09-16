package org.firstinspires.ftc.teamcode;

// Note:  Package information can be located under:
//       Rover-Ruckus/doc/javadoc/com/qualcomm/
//       Rover-Ruckus/doc/javadoc/org/firstinspires/ftc/robotcore/
//
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.internal.system.*;

/*
 *  Note:  The following Java imports are found directly from the JRE
 *         locations will vary depending on platform
 *         For OSX: /Library/Java/JavaVirtualMachines/jdk1.8.0_60.jdk/Contents/Home
 *         For WIN:  TBD....
 */
import java.io.File;
import java.util.Locale;


/*
 *  Created by: Eric Harwood on 9/16/18
 *
 *  Purpose:
 *    The purpose of this Hardware Map file is to define all the specific hardware for
 *    the 2018 Rover-Ruckus competition robot
 *
 *    Usage:
 *
 *    Within this file, you should define mappings for:
 *      - DC Motors
 *      - Servos
 *      - Sensors
 *      - USB WebCameras (if used)
 *      - Misc Mapped hardware (I2C, etc..)
 *
 *    Note:
 *     You may need to include additional "import" statements above to "pull-in" additional
 *     Java Libraries
 *
 *
 *
 *     Naming Conventions:
 *
 *     Please keep naming conventions simple.  This reduces debugging issues within the code and on the robotContoller mappings.
 *     Identify what they are used for and or position.
 *
 *     For example, a servo used for an ingest arm on the left should be named: leftIngest
 *
 *
 *
 *     Motors:
 *       - All motors should be labeled:   leftFront, rightfront, leftBack, rightBack
 *       - Any additional motors for arms or lifts can be named accordingly.
 *
 *
 */


public class HardwareCS {

    // Public OpMode Memebers

    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;


    //public Servo trigger = null;
    public Servo ServoLift = null;   //  <-----  THIS IS AN EXAMPLE ONLY.  PLEASE CHANGE ACCORDINGLY.......

    /* Local OpMode Members */
    HardwareMap hwMap     = null;
    private ElapsedTime period = new ElapsedTime();


    //~~~~ IMU:
    // public BNO055IMU imu;
    // public boolean calibratedIMU;

    // Color Range Sensor
    //public LynxI2cColorRangeSensor ColorRange1 = null; // the rev robotics color range sensor
    //public I2cAddr ColorNumber1 = I2cAddr.create7bit(0x39);// the address for the  I2c color range sensor

    //public LynxI2cColorRangeSensor Sensor_R = null;
    //public LynxI2cColorRangeSensor Sensor_L = null;

    //public I2cAddr Sensor_R_Number = I2cAddr.create7bit(0x39);
   // public I2cAddr Sensor_L_Number = I2cAddr.create7bit(0x39);

    //public LynxI2cColorRangeSensor BackIngest = null;
    //public LynxI2cColorRangeSensor FrontIngest = null;

    //public I2cAddr BackIngest_Number = I2cAddr.create7bit(0x39);
    //public I2cAddr FrontIngest_Number = I2cAddr.create7bit(0x39);

    //public LynxI2cColorRangeSensor sensor_BackIngest = null;
    //public LynxI2cColorRangeSensor sensor_FrontIngest = null;

    //public I2cAddr sensor_BackIngest_Number = I2cAddr.create7bit(0x39);
    //public I2cAddr sensor_FrontIngest_Number = I2cAddr.create7bit(0x39);

   // public LynxI2cColorRangeSensor ColorRange2 = null; // the rev robotics color range sensor
   //public I2cAddr ColorNumber2 = I2cAddr.create7bit(0x39);// the address for the  I2c color range sensor


    public void init (HardwareCS robot_CS) {

        // Save reference to Hardware Map
        hwMap = robot_CS;

        // Define and initialize motors

        leftFront = hwMap.dcMotor.get("leftFront");
        leftRear = hwMap.dcMotor.get("leftRear");
        rightFront = hwMap.dcMotor.get("rightFront");
        rightRear = hwMap.dcMotor.get("rightRear");



        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
=
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear/.setDirection(DcMotorSimple.Direction.FORWARD);

        rightFront.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        leftRear.setPower(0);

        // Trigger = hwMap.servo.get("trigger");

    }

    //------------------------------------------------------------
    // IMU - BNO055
    // Set up the parameters with which we will use our IMU.
    // + 9 degrees of freedom
    // + use of calibration file (see calibration program)
    //------------------------------------------------------------
    //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    //parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    //parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    //parameters.loggingEnabled = true;
    // parameters.loggingTag = "IMU";
    // parameters.mode = BNO055IMU.SensorMode.NDOF;

    //parameters.accelerationIntegrationAlgorithm = null;

    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    //imu = hwMap.get(BNO055IMU.class, "imu");
    //imu.initialize(parameters);

    // Read the IMU configuration from the data file saved during calibration.
    // Using try/catch allows us to be specific about the error instead of
    // just showing a NullPointer exception that could come from anywhere in the program.
    //calibratedIMU = true;
       /* try {
            File file = AppUtil.getInstance().getSettingsFile(parameters.calibrationDataFile);
            String strCalibrationData = ReadWriteFile.readFile(file);
            BNO055IMU.CalibrationData calibrationData = BNO055IMU.CalibrationData.deserialize(strCalibrationData);
            imu.writeCalibrationData(calibrationData);
        } catch (Exception e) {
            calibratedIMU = false;
        }*/
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

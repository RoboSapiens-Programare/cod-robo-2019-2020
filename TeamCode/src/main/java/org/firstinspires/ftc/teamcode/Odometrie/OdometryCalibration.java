package org.firstinspires.ftc.teamcode.Odometrie;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */

@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public abstract class OdometryCalibration extends RobotHardware {

    final double PIVOT_SPEED = 0.5;

    //Constanta scoasa de ei. Am convertit-o in centimetri. Va fi diferita la noi cel mai probabil pt ca folosim complet alte piese.
    final double COUNTS_PER_INCH = 307.699557; //caca-m-as in inch-ul lor
    final double COUNTS_PER_CM = 781.556875;

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        CalibrateGyro();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(getZAngle() < 90 && opModeIsActive()){

        }  MotorFR.setPower(-PIVOT_SPEED);
        MotorBR.setPower(-PIVOT_SPEED);
        MotorFL.setPower(PIVOT_SPEED);
        MotorBL.setPower(PIVOT_SPEED);
        if(getZAngle() < 60) {
            SetWheelsPowerTank(PIVOT_SPEED, -PIVOT_SPEED);
        }else{
            SetWheelsPowerTank(PIVOT_SPEED/2, -PIVOT_SPEED/2);
        }

        telemetry.addData("IMU Angle", getZAngle());
        telemetry.update();

       /* SetWheelsPowerTank(PIVOT_SPEED, -PIVOT_SPEED);

        while ( getZAngle() < 60 && opModeIsActive() ) {
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
            idle();
        }

        SetWheelsPowerTank(PIVOT_SPEED/2, -PIVOT_SPEED/2);
        while ( getZAngle() < 90 && opModeIsActive() ) {
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
            idle();
        }*/

        //Stop the robot
        StopWheels();
              timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getZAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(MotorVerticalLeft.getCurrentPosition()) + (Math.abs(MotorVerticalRight.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = MotorHorizontal.getCurrentPosition()/Math.toRadians(getZAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

            //Display raw values
            telemetry.addData("IMU Angle", getZAngle());
            telemetry.addData("Vertical Left Position", -MotorVerticalLeft.getCurrentPosition());
            telemetry.addData("Vertical Right Position", MotorVerticalRight.getCurrentPosition());
            telemetry.addData("Horizontal Position", MotorHorizontal.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getZAngle(){
        return (-Gyro.getAngularOrientation().firstAngle);
    }


}

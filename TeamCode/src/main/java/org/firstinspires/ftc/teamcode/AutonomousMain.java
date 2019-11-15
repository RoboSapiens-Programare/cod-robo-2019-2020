package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

public abstract class AutonomousMain extends RobotHardware {

    @Override
    public void initialize(){
        super.initialize();
        CalibrateGyro();
    }


    //SENSOR INPUT
    //Function that questions the selected sensor for the selected color
    protected boolean CheckForColor(ColorSensor SensorToQuestion, int HUE){


        return true;
    }

    //MOVEMENT
    protected void StrafeWithEncoders(double angle, double speed, double ticks){
//        //CONSTANTE
//        final double STRAIGHTTICKS = 0, STRAFETICKS = 0;

        ResetAllEncoders();

        RunWithAllEncoders();

        StrafeWithAngle(angle, 0, speed);

        double x = Math.cos(Math.toRadians(angle));
        double y = Math.sin(Math.toRadians(angle));

        CalculateMecanumResult(x , y);

        while(Math.abs(GetMotorEncodersAverage(new DcMotor[]{MotorFL, MotorBR})) < Math.abs(FLBRResult*ticks)
              || Math.abs(GetMotorEncodersAverage(new DcMotor[]{MotorFR, MotorBL})) < Math.abs(FRBLResult*ticks)){
            telemetry.addData("AverageL", GetMotorEncodersAverage(new DcMotor[]{MotorFL, MotorBR}));
            telemetry.addData("FLBR" , FLBRResult*ticks);
            telemetry.addData("AverageR", GetMotorEncodersAverage(new DcMotor[]{MotorFR, MotorBL}));
            telemetry.addData("FRBL" , FRBLResult*ticks);
            telemetry.update();

            idle();
        }

        StopWheels();

    }

    protected void StrafeWithObstacle(double targetDistance) throws InterruptedException{
        ElapsedTime ETimer = new ElapsedTime();
        ETimer.reset();

        double TOLERANCE = 0.001;
        double TOLERANCE_MAX = 200;

        double errorLeft = RangeL.getDistance(DistanceUnit.CM) - targetDistance;
        double errorRight = RangeR.getDistance(DistanceUnit.CM) - targetDistance;

        double pGain = 0.25/(targetDistance-5);
        int AverageCount = 8, AverageIterator = 0; //TODO modify
        double[] MeanArrayLeft = new double[AverageCount+1];
        double[] MeanArrayRight = new double[AverageCount+1];

        for (int i=0; i<AverageCount; i++){
            MeanArrayLeft[i] = errorLeft;
            MeanArrayRight[i] = errorRight;
        }

        double speedLeft;
        double speedRight;
        double dTime;

        long steadyTimer = 0, period = 10L, constTimer = 0;

        while (opModeIsActive() && steadyTimer < 2000) {
            double aux = ETimer.milliseconds();

            if(Math.abs(errorLeft) < 5 && Math.abs(errorRight) < 5){
                steadyTimer += period;
            }else{
                steadyTimer = 0;
            }

            //Update

            double DistLeft = RangeL.getDistance(DistanceUnit.CM);
            double DistRight = RangeR.getDistance(DistanceUnit.CM);

            errorLeft = Range.clip(DistLeft - targetDistance, -TOLERANCE_MAX, TOLERANCE_MAX);
            errorRight = Range.clip(DistRight - targetDistance, -TOLERANCE_MAX, TOLERANCE_MAX);

            //........s e n d   h e l p .......


            MeanArrayLeft[AverageIterator] = errorLeft;
            MeanArrayRight[AverageIterator] = errorRight;

            AverageIterator++;

            if(AverageIterator >= AverageCount) {
                //telemetry.addData("AverageIterator a fost mai mare decat AverageCount", "");
                AverageIterator = 0;
            }
            double meanLeft = 0, meanRight = 0;

            for(int i = 0;  i < AverageCount; i++){
                meanLeft += MeanArrayLeft[i];
                meanRight += MeanArrayRight[i];
            }


            speedLeft = Range.clip(meanLeft/AverageCount*pGain, -0.4, 0.4);
            speedRight = Range.clip(meanRight/AverageCount*pGain, -0.4, 0.4);

            if (Math.abs(speedLeft) < TOLERANCE) {
                speedLeft = 0;
                errorLeft = 0;
            }
            if (Math.abs(speedRight) < TOLERANCE) {
                speedRight = 0;
                errorRight = 0;
            }

            //Set Powers
            SetWheelsPowerTank(speedLeft, speedRight);

            dTime = ETimer.milliseconds() - aux;

            //telemetry.addData("RangeL", RangeL.getDistance(DistanceUnit.CM));
            //telemetry.addData("RangeR", RangeR.getDistance(DistanceUnit.CM));
            telemetry.addData("DistLeft" , DistLeft);
            telemetry.addData("DistRigth" , DistRight);
            telemetry.addData("ErrorL", errorLeft);
            telemetry.addData("ErrorR", errorRight);
            telemetry.addData("MeanL", meanLeft/AverageCount);
            telemetry.addData("MeanR", meanRight/AverageCount);
            telemetry.addData("SpeedL", speedLeft);
            telemetry.addData("speedR ", speedRight);
            //telemetry.addData("dTime", dTime);
            //telemetry.addData("Elapsed Time ", ETimer.milliseconds());
            telemetry.update();

            steadyTimer += period;
            constTimer += period;

            //Thread.sleep(period);
        }
    }

    protected void Rotate (double Angle) {

        double deadzone_big = 15;
        double deadzone_small = 5;
        double speed = 0.4;

        if (Angle < 0)
            speed = - speed;

        double TargetAngle = GetAngle() + Angle;

        SetWheelsPowerTank(-speed , speed);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_big) {
            telemetry.addData("sunt in modul normal", TargetAngle);
            telemetry.addData("m-am rotit pana la ", GetAngle());
            telemetry.update();
            idle();
        }

        SetWheelsPowerTank(-speed/2 , speed/2);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_small) {
            telemetry.addData("sunt in modul incet", " ");
            telemetry.addData("m-am rotit pana la ", GetAngle());
            telemetry.update();
            idle();
        }

        SetWheelsPowerTank(0,0);
    }

    private int GetMotorEncodersAverage(DcMotor[] arr){
        int MeanTicks = 0;
        int number = 0;

        for(DcMotor a : arr){
            MeanTicks += a.getCurrentPosition();
            number++;
        }

        MeanTicks = MeanTicks/number;


        return MeanTicks;
    }
}

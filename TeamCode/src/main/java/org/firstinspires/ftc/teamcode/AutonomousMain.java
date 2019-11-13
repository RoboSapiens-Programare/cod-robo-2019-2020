package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

public abstract class AutonomousMain extends RobotHardware {

    @Override
    public void initialize(){
        super.initialize();
        CalibrateGyro();
    }

    protected void StrafeWithEncoders(double angle, double speed, double ticks){
        //TODO this
    }

    protected void StrafeWithObstacle(double targetDistance) throws InterruptedException{
        ElapsedTime ETimer = new ElapsedTime();
        ETimer.reset();

        double errorLeft = RangeL.getDistance(DistanceUnit.CM) - targetDistance;
        double errorRight = RangeR.getDistance(DistanceUnit.CM) - targetDistance;

        double pGain = 0.25/(targetDistance - 5);
        int AverageCount = 16, AverageIterator = 0; //TODO modify
        double MeanArrayLeft[] = new double[AverageCount];
        double MeanArrayRight[] = new double[AverageCount];

        double speedLeft;
        double speedRight;
        double dTime;

        long steadyTimer = 0, period = 10L, constTimer = 0;

        while (opModeIsActive() && steadyTimer < 2000) {
            if(Math.abs(errorLeft) < 10 && Math.abs(errorRight) < 10){
                steadyTimer += period;
            }else{
                steadyTimer = 0;
            }

            //Update
            errorLeft = RangeL.getDistance(DistanceUnit.CM) - targetDistance;
            errorRight = RangeR.getDistance(DistanceUnit.CM) - targetDistance;


            //........s e n d   h e l p .......



            speedLeft = Range.clip(errorLeft*pGain, -0.6, 0.6);
            speedRight = Range.clip(errorRight*pGain, -0.6, 0.6);

            double aux = ETimer.milliseconds();
            MeanArrayLeft[AverageIterator++] = speedLeft;
            MeanArrayRight[AverageIterator++] = speedRight;

            if(AverageIterator >= AverageCount) {
                AverageIterator = 0;
            }
            double meanLeft = 0, meanRight = 0;

            for(int i = 0;  i < AverageCount; i++){
                meanLeft += MeanArrayLeft[i];
                meanRight += MeanArrayRight[i];
            }
            dTime = ETimer.milliseconds() - aux;


            //Set Powers
            SetWheelsPowerTank(meanLeft/AverageCount, meanRight/AverageCount);

            //telemetry.addData("RangeL", RangeL.getDistance(DistanceUnit.CM));
            //telemetry.addData("RangeR", RangeR.getDistance(DistanceUnit.CM));
            telemetry.addData("ErrorL", errorLeft);
            telemetry.addData("ErrorR", errorRight);
            telemetry.addData("dTime", dTime);
            telemetry.addData("Elapsed Time ", ETimer.milliseconds());
            telemetry.update();

            steadyTimer += period;
            constTimer += period;

            //Thread.sleep(period);
        }
    }

    protected void Rotate (double Angle) {

        double deadzone = 5;
        double speed = 0.4;

        if (Angle < 0)
            speed = - speed;

        double CurrentAngle = GetAngle();
        double TargetAngle = GetAngle() + Angle;

        if ( TargetAngle > 180) {
            TargetAngle -= 360;
        } else if (TargetAngle < - 180) {
            TargetAngle +=360;
        }


        while ( GetAngle() < TargetAngle - deadzone || GetAngle() > TargetAngle + deadzone) {
            SetWheelsPowerTank(speed, - speed);
        }
        SetWheelsPowerTank(0,0);
    }

}

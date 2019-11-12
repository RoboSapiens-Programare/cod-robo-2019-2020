package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;

public abstract class AutonomousMain extends RobotHardware {



    protected void StrafeWithEncoders(double angle, double speed, double ticks){
        //TODO this
    }

    protected void StrafeWithObstacle(double targetDistance) throws InterruptedException{
        ElapsedTime ETimer = new ElapsedTime();
        ETimer.reset();

        double errorLeft = RangeL.getDistance(DistanceUnit.CM) - targetDistance;
        double errorRight = RangeR.getDistance(DistanceUnit.CM) - targetDistance;

        double pGain = 1/(targetDistance - 10);

        double speedLeft;
        double speedRight;

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

            speedLeft = Range.clip(errorLeft*pGain, -0.6, 0.6);
            speedRight = Range.clip(errorRight*pGain, -0.6, 0.6);


            //Set Powers
            SetWheelsPowerTank(speedLeft, speedRight);

            //telemetry.addData("RangeL", RangeL.getDistance(DistanceUnit.CM));
            //telemetry.addData("RangeR", RangeR.getDistance(DistanceUnit.CM));
            //telemetry.addData("ErrorL", errorLeft);
            //telemetry.addData("ErrorR", errorRight);
            telemetry.addData("Time", constTimer);
            telemetry.addData("Elapsed Time ", ETimer.milliseconds());
            telemetry.update();

            steadyTimer += period;
            constTimer += period;

            Thread.sleep(period);
        }
    }

}

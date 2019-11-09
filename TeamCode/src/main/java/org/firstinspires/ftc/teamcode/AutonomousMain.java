package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class AutonomousMain extends RobotHardware {



    protected void StrafeWithEncoders(double angle, double speed, double ticks){
        //TODO this
    }

    protected void StrafeWithObstacle(double speed, double targetDistance){
        double errorLeft = -(targetDistance - RangeL.getDistance(DistanceUnit.CM));
        double errorRight = -(targetDistance - RangeR.getDistance(DistanceUnit.CM));

        double pGain = 1/(targetDistance - 5);

        double speedLeft = Range.clip(errorLeft*pGain, -0.9, 0.9);
        double speedRight = Range.clip(errorRight*pGain, -0.9, 0.9);

        long steadyTimer = 0, period = 10L;

        while (opModeIsActive() && steadyTimer < 2000) {
            if(errorLeft + errorRight < 10){
                steadyTimer += period;
            }else{
                steadyTimer = 0;
            }

            //Update
            errorLeft = -(targetDistance - RangeL.getDistance(DistanceUnit.CM));
            errorRight = -(targetDistance - RangeR.getDistance(DistanceUnit.CM));

            speedLeft = Range.clip(errorLeft*pGain, -0.9, 0.9);
            speedRight = Range.clip(errorRight*pGain, -0.9, 0.9);



            //Set Powers
            SetWheelsPowerTank(speedLeft, speedRight);

            sleep(period);
        }
    }

}

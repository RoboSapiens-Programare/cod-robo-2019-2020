package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.max;

public abstract class AutonomousMain extends RobotHardware {

    @Override
    public void initialize(){
        super.initialize();
        ServoClampLeft.setPosition(0.6);
        ServoClampRight.setPosition(0.4);

        CalibrateGyro();
    }

    //SENSOR INPUT
    //Check valid input
    protected boolean CheckHasLightInput(ModernRoboticsI2cColorSensor SensorToQuestion){
        return SensorToQuestion.alpha() > 1;
    }

    //Function that questions the selected sensor for the selected color
    protected boolean CheckForColor(ModernRoboticsI2cColorSensor SensorToQuestion, int HUE_A , int HUE_B){
        int COLOR_NOW = SensorToQuestion.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        if(COLOR_NOW >= HUE_A && COLOR_NOW <= HUE_B){
            return true;
        }else{
            return false;
        }
    }

    protected boolean CheckForColor(ModernRoboticsI2cColorSensor SensorToQuestion, int HUE_A , int HUE_B , int MIN , int MAX){
        int COLOR_NOW = SensorToQuestion.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
        int MAX_COLOR = Math.max(Math.max(SensorToQuestion.green() , SensorToQuestion.blue()) , SensorToQuestion.red());
        if (MAX_COLOR >= MIN && MAX_COLOR <= MAX && COLOR_NOW >= HUE_A && COLOR_NOW <= HUE_B){
            return true;
        }
        return false;
    }

    //MOVEMENT
    protected void StrafeWithEncoders(double angle, double speed, double ticks){

        ResetAllEncoders();

        RunWithAllEncoders();

        StrafeWithAngle(angle, 0, speed);

        angle += 90;

        double x = Math.cos(Math.toRadians(angle));
        double y = Math.sin(Math.toRadians(angle));

        CalculateMecanumResult(y , x);

        double TargetFLBR = FLBRResult;
        double TargetFRBL = FRBLResult;

        int AngleIterator = 0;

        while(opModeIsActive() && (Math.abs(GetMotorEncodersAverage(new DcMotor[]{MotorFL, MotorBR})) < Math.abs(TargetFLBR*ticks)
              || Math.abs(GetMotorEncodersAverage(new DcMotor[]{MotorFR, MotorBL})) < Math.abs(TargetFRBL*ticks))){
            telemetry.addData("AverageL", GetMotorEncodersAverage(new DcMotor[]{MotorFL, MotorBR}));
            telemetry.addData("FLBR" , TargetFLBR*ticks);
            telemetry.addData("AverageR", GetMotorEncodersAverage(new DcMotor[]{MotorFR, MotorBL}));
            telemetry.addData("FRBL" , TargetFRBL*ticks);
            telemetry.addData("Gyro reads:", GetAngle());
            telemetry.update();

            //AngleIterator++;
            if(AngleIterator >= 60) {
                RotateReset();
                StrafeWithAngle(angle, 0, speed);
                AngleIterator = 0;
            }

            sleep(1);
            idle();
        }

        //RotateReset();

        StopWheels();

    }

    protected void StrafeToObstacle(double targetDistance) {
        double TOLERANCE = 0.001;
        double TOLERANCE_MAX = 200;

        double errorLeft = RangeL.getDistance(DistanceUnit.CM) - targetDistance;
        double errorRight = RangeR.getDistance(DistanceUnit.CM) - targetDistance;

        double pGain = 0.0666;
        int AverageCount = 8, AverageIterator = 0; //TODO modify
        double[] MeanArrayLeft = new double[AverageCount + 1];
        double[] MeanArrayRight = new double[AverageCount + 1];

        for (int i = 0; i < AverageCount; i++) {
            MeanArrayLeft[i] = errorLeft;
            MeanArrayRight[i] = errorRight;
        }

        double speedLeft;
        double speedRight;

        int ResetAngle = 30;
        int AngleIterator = 0;

        while (opModeIsActive() && (Math.abs(errorLeft) > 5 || Math.abs(errorRight) > 5)) {
            //Update
            double DistLeft = RangeL.getDistance(DistanceUnit.CM);
            double DistRight = RangeR.getDistance(DistanceUnit.CM);

            errorLeft = Range.clip(DistLeft - targetDistance, -TOLERANCE_MAX, TOLERANCE_MAX);
            errorRight = Range.clip(DistRight - targetDistance, -TOLERANCE_MAX, TOLERANCE_MAX);

            MeanArrayLeft[AverageIterator] = errorLeft;
            MeanArrayRight[AverageIterator] = errorRight;

            AverageIterator++;
            AngleIterator++;

            if (AverageIterator >= AverageCount) {
                //telemetry.addData("AverageIterator a fost mai mare decat AverageCount", "");
                AverageIterator = 0;
            }

            if (AngleIterator >= ResetAngle){
                RotateReset();
                AngleIterator = 0;
            }

            double meanLeft = 0, meanRight = 0;

            for (int i = 0; i < AverageCount; i++) {
                meanLeft += MeanArrayLeft[i];
                meanRight += MeanArrayRight[i];
            }


            speedLeft = Range.clip(Math.pow(meanLeft / AverageCount * pGain, 3), -0.4, 0.4);
            speedRight = Range.clip(Math.pow(meanRight / AverageCount * pGain, 3), -0.4, 0.4);

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


            //telemetry.addData("RangeL", RangeL.getDistance(DistanceUnit.CM));
            //telemetry.addData("RangeR", RangeR.getDistance(DistanceUnit.CM));
            telemetry.addData("DistLeft", DistLeft);
            telemetry.addData("DistRigth", DistRight);
            telemetry.addData("ErrorL", errorLeft);
            telemetry.addData("ErrorR", errorRight);
            telemetry.addData("MeanL", meanLeft / AverageCount);
            telemetry.addData("MeanR", meanRight / AverageCount);
            telemetry.addData("SpeedL", speedLeft);
            telemetry.addData("speedR ", speedRight);
            //telemetry.addData("dTime", dTime);
            //telemetry.addData("Elapsed Time ", ETimer.milliseconds());
            telemetry.update();

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

        double pGain = 1/(targetDistance-5);
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


            speedLeft = Range.clip(Math.pow(meanLeft/AverageCount*pGain, 3), -0.4, 0.4);
            speedRight = Range.clip(Math.pow(meanRight/AverageCount*pGain, 3), -0.4, 0.4);

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

    protected void StrafeUntilColor(double speed, double angle, ModernRoboticsI2cColorSensor SensorToQuestion, int HUE_A , int HUE_B , boolean ResetAngle){
        StrafeWithAngle(angle, 0, speed);

        int AngleIterator = 0;
        int AngleReset = 60;

        while(!CheckForColor(SensorToQuestion, HUE_A , HUE_B) && opModeIsActive()){
            telemetry.addData("Color Read", SensorToQuestion.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();

            if(ResetAngle){
                AngleIterator++;
                if(AngleIterator >= AngleReset) {
                    RotateReset();
                    StrafeWithAngle(angle, 0, speed);
                    AngleIterator = 0;
                }
            }
            idle();
        }
        StopWheels();
    }

    protected void StrafeUntilColor(double speed, double angle, ModernRoboticsI2cColorSensor SensorToQuestion, int HUE_A , int HUE_B , int MIN , int MAX , boolean ResetAngle){
        StrafeWithAngle(angle, 0, speed);

        int AngleIterator = 0;
        int AngleReset = 60;

        while(!CheckForColor(SensorToQuestion, HUE_A , HUE_B , MIN , MAX) && opModeIsActive()){
            telemetry.addData("Color Read", SensorToQuestion.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();

            if(ResetAngle){
                AngleIterator++;
                if(AngleIterator >= AngleReset) {
                    RotateReset();
                    StrafeWithAngle(angle, 0, speed);
                    AngleIterator = 0;
                }
            }
            idle();
        }
        StopWheels();
    }

    protected void ResetAlpha(ModernRoboticsI2cColorSensor SensorToQuestion){
        StrafeWithAngle(-90, 0, 0.17);
        while (SensorToQuestion.alpha() < valAlphaMIN){
            idle();
        }
        StopWheels();
    }

    protected void StrafeUntilColorAlpha(double speed, double angle, ModernRoboticsI2cColorSensor SensorToQuestion, int HUE_A , int HUE_B , int MIN , int MAX){
        StrafeWithAngle(angle, 0, speed);

        while(!CheckForColor(SensorToQuestion, HUE_A , HUE_B , MIN , MAX) && opModeIsActive()){
            if (SensorToQuestion.alpha() < valAlphaMIN){
                ResetAngle();
                ResetAlpha(SensorToQuestion);
                StrafeWithAngle(angle, 0, speed);
            }
            telemetry.addData("Color Read", SensorToQuestion.read8(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
            telemetry.update();
            idle();
        }
        StopWheels();
    }

    protected void AddToGlobalTargetAngle(double addAngle){
        //globalTargetAngle %= 360;
        globalTargetAngle += addAngle;
        if (globalTargetAngle > 180){
            globalTargetAngle -= 360;
        }
        if (globalTargetAngle < -180){
            globalTargetAngle += 360;
        }
    }

    protected void RotateRelative(double Angle){

        AddToGlobalTargetAngle(Angle);

        double deadzone_big = 15;
        double deadzone_small = 5;
        double speed = 0.4;

        if (Angle < 0)
            speed = - speed;

        double TargetAngle = GetAngle() + Angle;

        SetWheelsPowerTank(-speed , speed);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_big) {
            idle();
        }

        SetWheelsPowerTank(-speed/2 , speed/2);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_small) {
            idle();
        }

        SetWheelsPowerTank(0,0);
    }

    protected void RotateRelative(double Angle , double deadzone_big , double deadzone_small , double speed){

        if (Angle < 0)
            speed = - speed;

        double TargetAngle = GetAngle() + Angle;

        SetWheelsPowerTank(-speed , speed);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_big) {
            idle();
        }

        SetWheelsPowerTank(-speed/2 , speed/2);

        while (opModeIsActive() && Math.abs(TargetAngle - GetAngle()) > deadzone_small) {
            idle();
        }

        SetWheelsPowerTank(0,0);
    }

    protected void RotateReset(){

        double ChangeAngle = globalTargetAngle - GetAngle();
        telemetry.addData("ChangeAngle" , ChangeAngle);
        telemetry.addData("globalTargetAngle" , globalTargetAngle);
        telemetry.addData("angle" , GetAngle());
        telemetry.update();
        RotateRelative(ChangeAngle , 7 , 2 , 0.3);

    }

    protected void AutonomousSkyStoneBlue(){
        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.7, 300); RotateReset();
        RotateRelative(-90);
        StrafeToObstacle(5); RotateReset();

        //Mergem pana gasim ceva de culoarea galbena
        RotateRelative(180);
        StrafeWithEncoders(180, 0.3, 150); sleep(100);
        StrafeWithEncoders(-90, 0.4, 350); sleep(100); RotateReset();
        StrafeUntilColor(0.17, -90, ColorSide, 1 , 16 , false); RotateReset();
        sleep(100);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColorAlpha(0.25, 0, ColorSide, 9 , 16 , 0 , 5);


        //Dam bratul in jos
        ServoBrat.setPosition(0.1);
        sleep(100);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.7, 900); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 2 , 4 , 1 , 300 , true); RotateReset();
        StrafeWithEncoders(0, 0.5, 300);

        //Ridicam bratul
        RotateRelative(90);
        ServoBrat.setPosition(1);
        sleep(200);
        RotateRelative(-90);

        StrafeWithEncoders(180, 0.5, 700); RotateReset();

        /*StrafeToObstacle(90);RotateReset();

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        RotateRelative(180);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.4, -90, ColorSide, 1 , 16); RotateReset();
        StrafeWithEncoders(90 , 0.2 , 20); RotateReset();

        sleep(100);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeWithAngle(-180, 0, 0.2);
        while(opModeIsActive()&& !CheckForColor(ColorSide, 9, 16) && CheckHasLightInput(ColorSide)){
            idle();
        }
        StopWheels(); RotateReset();

        sleep(200);

        //Dam bratul in jos
        StrafeWithEncoders(-90 , 0.2 , 25); RotateReset();
        ServoBrat.setPosition(0.1);
        sleep(200);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1500); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 2 , 4); RotateReset();

        //Ridicam bratul
        RotateRelative(90);
        sleep(100);
        ServoBrat.setPosition(1);
        sleep(100);*/
    }

    protected void AutonomousSkyStoneRed(){
        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.7, 300); RotateReset();
        RotateRelative(90);

        StrafeToObstacle(5);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeWithEncoders(-90, 0.4, 200); sleep(100);
        StrafeUntilColor(0.2, -90, ColorSide, 1 , 16 , false); RotateReset();
        sleep(200);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColorAlpha(0.25, 180, ColorSide, 9 , 16 , 0 , 5);
        //StrafeWithEncoders(0, 0.2, 20); RotateReset();

        sleep(200);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        sleep(200);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.7, 600); RotateReset();
        StrafeUntilColor(0.4, 180, ColorUnderneath, 9 , 11 , 5 , 300 , true); RotateReset();

        //Ridicam bratul
        RotateRelative(-90);
        StrafeWithEncoders(-90, 0.7, 800);
        sleep(300);
        ServoBrat.setPosition(1);
        RotateRelative(90);

        StrafeToObstacle(93); RotateReset();


        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.2, -90, ColorSide, 1 , 16 , false); RotateReset();

        sleep(200);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeWithAngle(0, 0, 0.3);
        while(opModeIsActive()&& !CheckForColor(ColorSide, 9, 16 , 0 , 10) && CheckForColor(ColorSide, 1, 16, 1, 300)){
            idle();
        }
        StopWheels();

        if (!CheckForColor(ColorSide, 1, 16, 1, 300)){
            telemetry.addData("Am vazut gol coaie", "");
            StrafeWithEncoders(-180, 0.2, 150); RotateReset();
        }
        else{
            //StrafeWithEncoders(0, 0.2, 80); RotateReset();
        }

        //123sleep(200);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        sleep (100);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 600); RotateReset();
        StrafeUntilColor(0.5, 180, ColorUnderneath, 9 , 11 , 5 , 300 , true); RotateReset();

        //Ridicam bratul
        RotateRelative(-90);
        StrafeWithEncoders(-90, 0.7, 800);
        sleep(100);
        ServoBrat.setPosition(1);
        StrafeUntilColor(0.5, 90, ColorUnderneath, 9 , 11 , 5 , 300 , true); RotateReset();

        /*//Ne dam mai in drepta
        StrafeWithEncoders(-90, 0.6, 600); RotateReset();

        //Mergem in fata si rotim sa fim cu senzorul inspre caramizi
        StrafeWithEncoders(0, 0.5, 500); RotateReset();
        RotateRelative(90);

        //Mergem pana gasim ceva de culoarea galbena
        StrafeUntilColor(0.2, -90, ColorSide, 8); RotateReset();

        *//*sleep(500);

        //Mergem in fata pana trecem de primul stone
        StrafeWithAngle(0, 0, 0.3);
        while(opModeIsActive() && CheckHasLightInput(ColorSide)){
            idle();
        }
        StopWheels(); RotateReset();*//*

        sleep(500);

        //Incepem sa le luam in ordine pana ajungem la unul negru
        StrafeUntilColor(0.15, -180, ColorSide, 11); RotateReset();
        StrafeWithEncoders(180, 0.2, 75); RotateReset();

        sleep(500);

        //Dam bratul in jos
        ServoBrat.setPosition(0.1);

        //Tragem ala in building zone
        StrafeWithEncoders(90, 0.6, 1000); RotateReset();
        StrafeUntilColor(0.5, 0, ColorUnderneath, 3); RotateReset();
        sleep(300);
        //Ridicam bratul
        ServoBrat.setPosition(1);*/
    }

    protected void AutonomousImpingeTavaBlue(){
        // EXPERIMENTAL

        //Merg pana la 40 de cm de perete
        StrafeWithEncoders(0, 0.5, 200); RotateReset();
        RotateRelative(90);
        sleep(100);
        StrafeToObstacle(55); //TODO

        //Merg pana la tava si o apuc
        StrafeWithEncoders(-90, 0.5, 1000); RotateReset();
        ServoBrat.setPosition(0.1);
        sleep(200);

        //Trag tava si ma desprind
        StrafeWithEncoders(70, 0.6, 3000);
        ServoBrat.setPosition(1); RotateReset();
        sleep(200);

        //Ocolesc tava
        StrafeToObstacle(100);
        StrafeWithEncoders(-90 , 0.5 , 1200); RotateReset();
        StrafeWithEncoders(0 , 0.5 , 200); RotateReset();
        StrafeWithEncoders(-90 , 0.5 , 1200); RotateReset();
        StrafeToObstacle(30);
        StrafeWithEncoders(90 , 0.8 , 2500); RotateReset();

        //Parchez
        StrafeUntilColor(0.5, -180, ColorUnderneath, 2 , 4 , true);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie
    }

    protected void AutonomousImpingeTavaRed(){
        // EXPERIMENTAL

        //Mers pana la 40 de cm de perete
        StrafeWithEncoders(0, 0.5, 200); RotateReset();
        RotateRelative(-90);
        sleep(100);
        StrafeToObstacle(20); //TODO

        //Ma rotesc ca sa am clestele orientat spre tava si merg pana la tava si o apuc
        RotateRelative(180); sleep(100); RotateReset();
        StrafeWithEncoders(-90, 0.5, 1300); RotateReset();
        ServoBrat.setPosition(0.1);
        sleep(200);

        //Trag tava si ma desprind si ma rotesc inapoi
        StrafeWithEncoders(90, 0.8, 2500);
        ServoBrat.setPosition(1); RotateReset();
        sleep(200);
        RotateRelative(180);

        //Ocoleste tava
        StrafeWithEncoders(180, 0.5, 500); RotateReset();
        StrafeToObstacle(100);
        StrafeWithEncoders(90 , 0.5 , 1000); RotateReset();
        StrafeWithEncoders(0 , 0.5 , 100); RotateReset();
        StrafeWithEncoders(90 , 0.5 , 1200); RotateReset();
        StrafeToObstacle(40);
        StrafeWithEncoders(-90 , 0.8 , 2200); RotateReset();

        //Parcheaza
        StrafeUntilColor(0.5, -180, ColorUnderneath, 9 ,11, 5, 300 , true);


        //dupa ce trag tava, o ocolesc, intai la dreapta
        //apoi in fata, apoi la stanga, apoi in spate

        //dupa ce am impins tava, ma duc in dreapta
        // ma duc in spate pana la zid (senzori si poate ne rotim 180)
        //dupa mergem la dreapta pana la linie
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class zip_tie_ma_tai extends LinearOpMode {
    public DcMotorEx slider1,slider2;
    public Pid_Controller_Adevarat pidS = new Pid_Controller_Adevarat(0,0,0);
    //public double Rotatie = 0, ticksPerDegree = Pid_Controller_TestConfig.rotationCalib;
    public int verifications = 0;
    public int totalS1 = 0;
    public double correctionS1;
    public double pows,  max;
    public double encDr, encSt, encSp;
    public boolean startTh = false;
    public double S;
    public zip_tie_ma_tai(boolean startThreads) {startTh = startThreads;}
    public void Init(HardwareMap hard) {
        slider1 = hardwareMap.get(DcMotorEx.class, "slider1");
        slider2 = hardwareMap.get(DcMotorEx.class, "slider2");

        
        slider2.setPower(0);
        slider1.setPower(0);

        slider1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slider1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        slider1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        
        pidS.setSetpoint(0);

        pidS.setTolerance(Config.tolerance);

        pidS.setPID(Config.ps, Config.is, Config.ds);

        pidS.enable();
    }
    private void powerY(double pows) {
        slider2.setPower(pows);
        slider1.setPower(pows);
    }


    public void slider(double incrementalY, double maxPow){
        totalS1 += incrementalY;
        //totalS1 = (int)incrementalY;
        verifications = 0;

        pidS.enable();
        

        pidS.setSetpoint(totalS1);


        pidS.setTolerance(Config.tolerance);

        
        do{
            S = (encDr + encSt)/2;
            
            correctionS1 = -pidS.performPID(S);

            pows =  correctionS1;
            

            max = Math.abs(pows);
            max = Math.abs(pows) > max ? Math.abs(pows) : max;
           

            if (max > maxPow) {
                pows /= max;
               
            }
            powerY(pows);
            verifications = pidS.onTarget() ? verifications + 1 : 0;
        }while(verifications < Config.targetVerifications);
        powerY(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
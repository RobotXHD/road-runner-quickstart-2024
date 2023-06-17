package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FunctiiDeAutonom extends LinearOpMode {
    public DcMotorEx motorFL,motorBR,motorFR,motorBL;
    public Pid_Controller_Adevarat pidRotatie = new Pid_Controller_Adevarat(0,0,0);
    public Pid_Controller_Adevarat pidY = new Pid_Controller_Adevarat(0,0,0);
    public Pid_Controller_Adevarat pidX = new Pid_Controller_Adevarat(0,0,0);
    public double Rotatie = 0, ticksPerDegree = Config.rotationCalib;
    public int verifications = 0;
    public int totalY = 0, totalX = 0, totalRot = 0;
    public double correctionR,correctionY,correctionX;
    public double ds, df, ss, sf, Y, tempRot, max;
    public double encDr, encSt, encSp;
    public boolean startTh = false;
    public FunctiiDeAutonom(boolean startThreads) {startTh = startThreads;}
    public void Init(HardwareMap hard) {
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR");
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL");
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR");

        motorFR.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorBL.setPower(0);

        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidRotatie.setSetpoint(0);
        pidY.setSetpoint(0);
        pidX.setSetpoint(0);

        pidRotatie.setTolerance(Config.toleranceRotatie);
        pidY.setTolerance(Config.toleranceY);
        pidX.setTolerance(Config.toleranceX);

        pidRotatie.setPID(Config.p, Config.i, Config.d);
        pidY.setPID(Config.py, Config.iy, Config.dy);
        pidX.setPID(Config.px, Config.ix, Config.dx);

        pidRotatie.enable();
        pidY.enable();
        pidX.enable();
    }
    private void powerY(double ds, double df, double ss, double sf) {
        motorFR.setPower(df);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
        motorBL.setPower(ss);
    }

    private void powerX(double ds, double df, double ss, double sf) {
        motorBL.setPower(ss);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
        motorFR.setPower(df);
    }

    private void powerRot(double ds, double df, double ss, double sf) {
        motorFR.setPower(df);
        motorBL.setPower(ss);
        motorFL.setPower(sf);
        motorBR.setPower(ds);
    }
    public void gotoX(double incrementalX, double maxPow){
        totalX += incrementalX;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(Config.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerX(ds, df, ss, sf);
            verifications = pidX.onTarget() ? verifications+1 : 0;
        }while(verifications < Config.targetVerifications);
        powerX(0,0,0,0);
    }
    public void gotoY(double incrementalY, double maxPow){
        totalY += incrementalY;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(Config.toleranceRotatie);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df) : max;
            max = Math.abs(sf) > max ? Math.abs(sf) : max;
            max = Math.abs(ss) > max ? Math.abs(ss) : max;

            if (max > maxPow) {
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerY(ds, df, ss, sf);
            verifications = pidY.onTarget() ? verifications + 1 : 0;
        }while(verifications < Config.targetVerifications);
        powerY(0,0,0,0);
    }
    public void rotatie(double incrementalRot, double maxPow, double tolerance){
        totalRot+=incrementalRot;
        verifications = 0;

        pidY.enable();
        pidX.enable();
        pidRotatie.enable();

        pidX.setSetpoint(totalX);
        pidY.setSetpoint(totalY);
        pidRotatie.setSetpoint(totalRot);

        pidX.setTolerance(Config.toleranceX);
        pidY.setTolerance(Config.toleranceY);
        pidRotatie.setTolerance(tolerance);
        do{
            Y = (encDr + encSt)/2;
            correctionR = -pidRotatie.performPID(Rotatie);
            correctionY = -pidY.performPID(Y);
            correctionX = pidX.performPID(encSp);
            ds = correctionR + correctionY - correctionX;
            df = correctionR + correctionY + correctionX;
            ss = -correctionR + correctionY + correctionX;
            sf = -correctionR + correctionY - correctionX;

            max = Math.abs(ds);
            max = Math.abs(df) > max ? Math.abs(df):max;
            max = Math.abs(sf) > max ? Math.abs(sf):max;
            max = Math.abs(ss) > max ? Math.abs(ss):max;
            if(max > maxPow){
                ds /= max;
                df /= max;
                sf /= max;
                ss /= max;
            }
            powerRot(ds, df, ss, sf);
            verifications = pidRotatie.onTarget() ? verifications+1 : 0;
        }while (verifications < Config.targetVerifications);
        powerRot(0,0,0,0);
    }
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
package org.firstinspires.ftc.teamcode.robonauts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp
@Config
public class PIDSlidesTuning extends OpMode {
    private PIDController pidController=null;
    public static  double kp=0.0012;//0.77;
    public static  double ki=0;//0.003;
    public static  double kd=0;//0.004;

    public static double kf=-0.0003;//0.03;

    public static int targetMotorPosition=-2000;

    //public static int targetDeg=0;
        private final double ticksPerDegree=537.7 / 360;
    public Encoder armMotorEncoder;
    DcMotorEx leftMotor = null;
    DcMotorEx rightMotor = null;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pidController = new PIDController(kp, ki,kd);

        leftMotor = hardwareMap.get(DcMotorEx.class, "par0");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hardwareMap.get(DcMotorEx.class, "par1");


        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        pidController.setPID(kp,ki,kd);
        double feedforward = kf;


        double rightPid = pidController.calculate(rightMotor.getCurrentPosition(), targetMotorPosition);
        double rightpower = rightPid + feedforward;

        double leftPid = pidController.calculate(leftMotor.getCurrentPosition(), targetMotorPosition);
        double leftPower = leftPid + feedforward;

        leftMotor.setPower(rightpower);
        rightMotor.setPower(rightpower);
        telemetry.addData("Feed Forward : " , feedforward);
        //telemetry.addData("Left Power : " , leftpower);
        telemetry.addData("right Power : " , rightpower);

        telemetry.addData("Left slide current position : " , leftMotor.getCurrentPosition());
        telemetry.addData("Right slide current position : ", rightMotor.getCurrentPosition());
        telemetry.addData("target position : ", targetMotorPosition);
        telemetry.update();
    }
}

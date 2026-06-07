package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BabyBopBot {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intake;

    private DcMotor backintake;
    private Servo feederleft;
    private Servo feederright;
    private Servo kicker;
    private DcMotorEx shooter;
    private SparkFunOTOS sparkFunOTOS;

    IMU imu;

    public void init(HardwareMap hardwareMap){
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        sparkFunOTOS = hardwareMap.get(SparkFunOTOS.class, "otos");
        configureOTOS();


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front left");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front right");
        backRightDrive = hardwareMap.get(DcMotor.class, "back right");

        intake = hardwareMap.get(DcMotor.class, "intake");
        backintake = hardwareMap.get(DcMotor.class,"back intake");
        //Vineeth made the change
        feederleft = hardwareMap.get(Servo.class, "feeder left");
        feederright = hardwareMap.get(Servo.class, "feeder right");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.FORWARD);

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        backintake.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // shooter.setMode(DcMotor.RunMode.);

        // Change coefficients using methods included with DcMotorEx class.
        PIDFCoefficients pidfNew = new PIDFCoefficients(150, 0, 0, 13.6);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        backintake.setPower(0);
        intake.setPower(-1);
        shooter.setVelocity(0);
    }
    private  void configureOTOS() {
        sparkFunOTOS.setLinearUnit(DistanceUnit.INCH);
        sparkFunOTOS.setAngularUnit(AngleUnit.DEGREES);
        sparkFunOTOS.setOffset(new SparkFunOTOS.Pose2D(-0.125, -3.75, 0));
        sparkFunOTOS.setLinearScalar(1.0);
        sparkFunOTOS.setAngularScalar(0.993);
        sparkFunOTOS.resetTracking();
        sparkFunOTOS.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
        sparkFunOTOS.calibrateImu(255, false);
    }
    public void setShooterVelocity(double shooterVelocity){
        shooter.setVelocity(shooterVelocity);
    }
    public double getShooterVelocity(){
        return shooter.getVelocity();
    }
    public double getShooterPower(){
        return shooter.getPower();
    }
    public double getHeading(){
       return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading(){
        imu.resetYaw();
    }
    public void setDrive(double fl, double fr, double bl, double br){
        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
    public void setFIntakeInward(){
        intake.setPower(-1);
    }
    public void setFIntakeBackward(){
        intake.setPower(1);
    }
    public void setFIntakeoff(){
        intake.setPower(0);
    }
    public void setBIntakeInward(){
        backintake.setPower(-1);
    }
    public void setBIntakeBackward(){
        backintake.setPower(1);
    }
    public void setBIntakeoff(){
        backintake.setPower(0);
    }
    public void kickerKick(){
        kicker.setPosition(0.5);
    }
    public void kickerIdle(){
        kicker.setPosition(0.2);
    }
    public void gateOpen(){
        feederleft.setPosition(0.15);
        feederright.setPosition(0.72);
    }
    public void gateClose(){
        feederleft.setPosition(0.3);
        feederright.setPosition(0.53);
    }
    public SparkFunOTOS.Pose2D getPosition(){
        return sparkFunOTOS.getPosition();
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp (name = "fieldCentricDrive")

public class fieldCentricDrive extends LinearOpMode {

    double speed;
    double slowSpeed = 0.4;
    double normalSpeed = 0.75;
    double fastSpeed = 1;

    DcMotor fL;
    DcMotor fR;
    DcMotor bL;
    DcMotor bR;
    CRServo rMolar;
    CRServo lMolar;

    IMU imu;
    IMU.Parameters myIMUparameters;
    YawPitchRollAngles robotOrientation;

    double yaw;
    double pitch;
    double roll;


    @Override
    public void runOpMode() {
        fL = hardwareMap.get(DcMotor.class, "SL");
        fR = hardwareMap.get(DcMotor.class, "SR");
        bL = hardwareMap.get(DcMotor.class, "IL");
        bR = hardwareMap.get(DcMotor.class, "IR");
        rMolar = hardwareMap.get(CRServo.class, "rightMolar");
        lMolar = hardwareMap.get(CRServo.class, "leftMolar");
        imu = hardwareMap.get(IMU.class, "imu");

        fL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        rMolar.setDirection(CRServo.Direction.REVERSE);


        speed = normalSpeed;


        myIMUparameters = new IMU.Parameters
                (
                        new RevHubOrientationOnRobot
                                (
                                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                                )
                );
        imu.initialize(myIMUparameters);
        robotOrientation = imu.getRobotYawPitchRollAngles();


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            yaw = robotOrientation.getYaw(AngleUnit.DEGREES);
            pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
            roll = robotOrientation.getRoll(AngleUnit.DEGREES);
            if (gamepad1.x)
            {
                speed = normalSpeed;
            } else if (gamepad1.left_trigger > 0)
            {
                speed = slowSpeed;
            } else if (gamepad1.right_trigger > 0)
            {
                speed = fastSpeed;
            }

            if(gamepad1.right_bumper)
            {
                imu.resetYaw();
            }

            if(gamepad2.dpad_up)
            {
                lMolar.setPower(1);
                rMolar.setPower(1);
            } else if (gamepad2.dpad_down)
            {
                lMolar.setPower(-1);
                rMolar.setPower(-1);
            } else
            {
                lMolar.setPower(0);
                rMolar.setPower(0);
            }
            drive();
        }
    }

    public void drive()
    {
        double forward = -gamepad1.left_stick_y * speed;
        double strafe = gamepad1.left_stick_x * speed;
        double pivot = gamepad1.right_stick_x * speed;

        double botHeading = robotOrientation.getYaw(AngleUnit.RADIANS);
        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        double denominator = Math.max((Math.abs(rotX)+Math.abs(rotY)+Math.abs(pivot)) * speed, 1);

        fL.setPower(((rotY + rotX + pivot)/denominator));
        fR.setPower(((rotY - rotX - pivot)/denominator));
        bL.setPower(((rotY - rotX + pivot)/denominator));
        bR.setPower(((rotY + rotX - pivot)/denominator));

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}


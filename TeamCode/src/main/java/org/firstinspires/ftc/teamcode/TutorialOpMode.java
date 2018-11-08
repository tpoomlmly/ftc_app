package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TutorialOpMode extends LinearOpMode {
    private Gyroscope imu;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private DistanceSensor sensorColorRange;
    private Servo servoTest;

    @Override
    public void runOpMode() {
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motorTest = hardwareMap.get(DcMotor.class, "motor 0 HD hex");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch sensor");
        sensorColorRange = hardwareMap.get(DistanceSensor.class, "rev colour");
        servoTest = hardwareMap.get(Servo.class, "srs0");

        telemetry.addData("Status", "Initialised");
        telemetry.update();

        waitForStart();

        double tgtPower = 0;
        while(opModeIsActive()) {
            tgtPower = this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);
            telemetry.addData("Target power", tgtPower);
            telemetry.addData("Motor power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

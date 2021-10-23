package org.firstinspires.ftc.blackswan;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TeleopRed")

public class TeleopRed extends LinearOpMode {

    double MAX_SPEED = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeft, backLeft, frontRight,backRight;
        CRServo carousel;

        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        backLeft = hardwareMap.get(DcMotor.class,"backLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backRight = hardwareMap.get(DcMotor.class,"backRight");

        carousel = hardwareMap.get(CRServo.class, "carousel");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //telemetry testing delete later!!!
        int detection = 0;
        while(opModeIsActive()) {
            //turn with right stick
            telemetry.addData("left stick value x", gamepad1.left_stick_x);
            telemetry.addData("left stick value y", gamepad1.left_stick_y);
            telemetry.addData("detection", detection);
            telemetry.update();
            if (gamepad1.right_stick_x > 0.1) {
                telemetry.addData("positive", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.right_stick_x < -0.1) {
                telemetry.addData("negative", gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
            } else if (gamepad1.left_stick_x < -0.4 && gamepad1.left_stick_y < -0.4){
                //move UpLeft
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = 1;
            } else if (gamepad1.left_stick_x > 0.4 && gamepad1.left_stick_y < -0.4){
                //move UpRight
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = 2;
            } else if (gamepad1.left_stick_x > 0.4 && gamepad1.left_stick_y > 0.4){
                //move DownRight
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
                backRight.setPower(0);
                frontLeft.setPower(0);
                detection = 3;
            } else if (gamepad1.left_stick_x > -0.4 && gamepad1.left_stick_y > 0.4){
                //move DownLeft
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
                backLeft.setPower(0);
                frontRight.setPower(0);
                detection = 4;
            } else if (gamepad1.left_stick_y > 0.1){
                //move Up
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = 5;
            } else if (gamepad1.left_stick_y < -0.1){
                //move Down
                frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
                backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
                detection = 6;
            } else if (gamepad1.left_stick_x > 0.1){
                //move Right
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED* -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = 7;
            } else if (gamepad1.left_stick_x < -0.1){
                //move Left
                frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
                frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
                backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
                detection = 8;
            }else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                detection = 0;
            }


            telemetry.update();

            turnDuck(carousel);
        }

    }

    protected void turnDuck(CRServo carousel){
        if(gamepad2.right_bumper){
            carousel.setPower(0.9);
        } else {
            carousel.setPower(0);
        }
    }
}

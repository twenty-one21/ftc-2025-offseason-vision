/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.HardwareMechanum;


/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TwoControllers", group="Linear Opmode")
public class TwoControllers extends LinearOpMode {
    //HardwareMechanum robot = new HardwareMechanum();

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null; //motor 3
    private DcMotor leftBackDrive = null; //motor 2
    private DcMotor rightBackDrive = null; //motor 1
    private DcMotor rightFrontDrive = null; //motor 0
    private DcMotor shoulderLeft = null;
    private DcMotor shoulderRight = null;
    private DcMotor forearm = null;
    private Servo servo = null;


    //Wrist stuff
    final double WRIST_INCREMENT = 0.001;
    final double WRIST_MAX = 1.0;
    final double WRIST_MIN = 0.25;
    double servoPosition = 0.0;

    //Finger stuff
    //bro i didn't know that's what he was saying to me as i was
    //going to my room bc i was cold :skull:
    //i hate to do this to him but i do have to comment this out
    //servo work better if it dont work like this
    // final double FINGER_INCREMENT = 0.002;
   //  final double CLAW_MAX = 1.0;
    final double CLAW_MIN = 0.8;
    // double leftFingerPosition = 0.0;
    // double rightFingerPosition = 0.0;

    double armPosition;
    double MIN_POSITION = 0;
    double MAX_POSITION = 1;
    double armOffset = 0;
    double ARM_SPEED = 0.02;

    @Override
    public void runOpMode() {

        //robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //motor 3
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive"); //motor 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //motor 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive"); //motor 0
        shoulderLeft = hardwareMap.get(DcMotor.class, "shoulder_left"); //motor 0 expansion hub
        shoulderRight = hardwareMap.get(DcMotor.class, "shoulder_right"); //motor 1 expansion hub
        forearm = hardwareMap.get(DcMotor.class, "forearm"); //motor 2 expansion hub
        servo = hardwareMap.get(Servo.class, "servo"); //servo 1 on control hub

/** ########################################################################################
 * !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
 * ########################################################################################
 * Most robots need the motors on one side to be reversed to drive forward.
 * The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
 * If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
 * that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
 * when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
 * Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
 * Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
 **/
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //lift init stuff
        shoulderLeft.setDirection(DcMotor.Direction.FORWARD);
        shoulderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulderRight.setDirection(DcMotor.Direction.REVERSE);
        shoulderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        forearm.setDirection(DcMotor.Direction.FORWARD);
        forearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hardwareMap.get(Servo.class, "servo");


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            double Shoulder_Forward = gamepad2.right_trigger;
            double Shoulder_Backward = -gamepad2.left_trigger;
            double Forearm_Movement = gamepad2.right_stick_y*-1;
            double  Claw_Position = gamepad2.left_stick_x;



            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double upperArmPower   = Shoulder_Forward + Shoulder_Backward;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            //claw
            if (Claw_Position < 0){
                Claw_Position = 0;;
            }
            double calculated_claw = Claw_Position * (1-CLAW_MIN) + CLAW_MIN;
            servo.setPosition(calculated_claw);

            // wheel power
            if (max > 1.0) {
                leftFrontPower  = leftFrontPower/max;
                rightFrontPower = rightFrontPower/max;
                leftBackPower   = leftBackPower/max;
                rightBackPower  = rightBackPower/max;
            }
        //shoulder power
            shoulderLeft.setPower(upperArmPower);
            shoulderRight.setPower(upperArmPower);
            //Forearm Power
            forearm.setPower(Forearm_Movement);

            //up
//            if(gamepad2.right_trigger > 0){
//                shoulderLeft.setPower(gamepad2.right_trigger*0.6);
//                shoulderRight.setPower(gamepad2.right_trigger*0.6);
//            }if (gamepad2.left_trigger > 0){
//                shoulderLeft.setPower(-gamepad2.left_trigger*0.6);
//                shoulderRight.setPower(-gamepad2.left_trigger*0.6);
//            }
//
//            //down
//            if(gamepad2.left_trigger > 0.1){
//                shoulderLeft.setPower(-gamepad2.left_trigger*0.6);
//                shoulderRight.setPower(gamepad2.left_trigger*0.6);
//            }if (gamepad2.left_trigger < 0.1){
//                shoulderLeft.setPower(0);
//                shoulderRight.setPower(0);
//            }
//
//            //lift
//            if(gamepad2.left_stick_y > 0.1){
//                forearm.setPower(gamepad2.left_stick_y*1.0);
//            }if (gamepad2.left_stick_y < 0.1){
//                forearm.setPower(0);
//            }

            //Code for the wrist
//            if (gamepad2.right_bumper) {
//                if (Double.compare(WRIST_MAX, servoPosition) >= 0){
//                    servoPosition += WRIST_INCREMENT;
//                    servo.setPosition(servoPosition);
//                }
//            }
//            if (gamepad2.left_bumper) {
//                if (Double.compare(WRIST_MIN, servoPosition) <= 0){
//                    servoPosition -= WRIST_INCREMENT;
//                    servo.setPosition(servoPosition);
//                }
//            }


            //Code for the claw
            if(gamepad2.b) {
            }
            if(gamepad2.a){
            }

            //Code for the claw
            if(gamepad2.x) {
            }
            if(gamepad2.y){
            }

            // //Code for the fingers
            // if(gamepad2.y) {
            //     //yeah yeah yeah don't copy-paste code
            //     //but you should also not stay up until 5 am on a school night working on code
            //     //and yet here we are

            //sorry conner

            //     if (Double.compare(FINGER_MAX, leftFingerPosition) >= 0){
            //         leftFingerPosition += FINGER_INCREMENT;
            //         leftFinger.setPosition(leftFingerPosition);
            //     }
            // }

            // if(gamepad2.x){
            //     if (Double.compare(FINGER_MIN, leftFingerPosition) <= 0){
            //         leftFingerPosition -= FINGER_INCREMENT;
            //         leftFinger.setPosition(leftFingerPosition);
            //     }
            // }

            // if(gamepad1.b) {
            //     if (Double.compare(FINGER_MAX, rightFingerPosition) >= 0){
            //         rightFingerPosition += FINGER_INCREMENT;
            //         rightFinger.setPosition(rightFingerPosition);
            //     }
            // }
            // if(gamepad2.a){
            //     if (Double.compare(FINGER_MIN, rightFingerPosition) <= 0){
            //         rightFingerPosition -= FINGER_INCREMENT;
            //         rightFinger.setPosition(rightFingerPosition);
            //     }
            // }



            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower / 2);
            rightFrontDrive.setPower(rightFrontPower / 2);
            leftBackDrive.setPower(leftBackPower / 2);
            rightBackDrive.setPower(rightBackPower / 2);
//
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("shoulderPower ", "%4.2f", upperArmPower);
            telemetry.addData("Wrist position", "%4.2f", servoPosition);
            telemetry.addData("Forearm Movement ", "%4.2f", Forearm_Movement);
            // telemetry.addData("Left finger position", "%4.2f", leftFingerPosition);
            // telemetry.addData("Right finger position", "%4.2f", rightFingerPosition);
            telemetry.update();

            //sleep(50);
        }


    }}

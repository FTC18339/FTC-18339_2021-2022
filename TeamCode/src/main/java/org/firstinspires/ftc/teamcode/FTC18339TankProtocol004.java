package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class FTC18339TankProtocol004 extends Main002 {
    private double left_front_power = 0;
    private double left_back_power = 0;
    private double right_front_power = 0;
    private double right_back_power = 0;

    private double moveMultiplier = 1f;

    int dir = 1;
    double adder = 0;

    public double[] currentCraneCoordinates = new double[] {0,0};

    public double[] groundPosition = new double[] {207.24, 164.24};

    @Override
    public void runOpMode() {

        //Initialize all hardware, servos, and sensors, as well as the algorithm
        initMaths();
        initHardware();
        initManualModes();

        //wait until the play button in pressed
        waitForStart();

        //Initial Position
        float q1 = 0f; float q2 = 1f; float q3 = 0f;

        base_arm_joint.setPosition(1 - q1);
        second_arm_joint.setPosition(1 - (q2 / (3/2)));
        hand.setPosition(1 - (q3 / (3/2)));

        //Is the program started?
        if (opModeIsActive()) {
            //While the program is started.
            while (opModeIsActive()) {
                // Show motor info on android phone

                //Runtime Loop to update hardware with input, and automation
                SetMotorForces();
                SetSpinnerForces();
                SetCraneIKTarget();
                SetGripperForces();

                //go to the ground position
                /*if(gamepad2.x) {
                    double[] qg = math.IKArm(groundPosition[0], -groundPosition[1], Math.PI / 2);

                    if(qg == null || Double.isNaN(qg[0]) || Double.isNaN(qg[1]) || Double.isNaN(qg[2])) {
                    } else {
                        base_arm_joint.setPosition(math.Clamp(1 - (qg[0] / (Math.PI)), 0, 1));
                        second_arm_joint.setPosition(math.Clamp(1 - (qg[1] / (3 * Math.PI / 2)), 0, 1));
                        hand.setPosition(math.Clamp(qg[2] / (3 * Math.PI / 2), 0, 1));

                        currentX = -groundPosition[0];
                        currentY = -groundPosition[1];
                        currentPhi = Math.PI / 2;
                    }
                }

                if(gamepad2.y) {
                    double[] qr = math.IKArm(math.startPosition.x, math.startPosition.y, math.startPosition.z);

                    if(qr == null || Double.isNaN(qr[0]) || Double.isNaN(qr[1]) || Double.isNaN(qr[2])) {
                    } else {
                        base_arm_joint.setPosition(math.Clamp(1 - (qr[0] / Math.PI), 0, 1));
                        second_arm_joint.setPosition(math.Clamp(1 - (qr[1] / (3 * Math.PI / 2)), 0, 1));
                        hand.setPosition(math.Clamp(1 - (qr[2] / (3 * Math.PI / 2)), 0, 1));

                        currentX = math.startPosition.x;
                        currentY = math.startPosition.y;
                        currentPhi = math.startPosition.z;
                    }
                }*/

                //Update phone info
                telemetry.update();
                idle();
            }
        }
    }

    boolean wasMovingLastFrame = false;
    boolean notRunningConveyor = true;
    boolean notRunningRoller = true;
    public void SetMotorForces() {
        if(!NoNullHardware()) return;

        //Algorithm determined wheel forces with the inputs
        double y = gamepad1.left_stick_y;

        //Right trigger right rotation
        //Right trigger is right rotation, left trigger is left. Can slow rotations by depressing trigger less.
        float rAxis = gamepad1.right_trigger - gamepad1.left_trigger;

        //find the speeds of each wheel
        left_front_power = math.GetWheelForceTank(y, 1, rAxis);
        left_back_power = math.GetWheelForceTank(y, 2, rAxis);
        right_front_power = math.GetWheelForceTank(y, 3, rAxis);
        right_back_power = math.GetWheelForceTank(y, 4, rAxis);

        //Show the powers on the telemetry screen
        telemetry.addData("y", gamepad1.left_stick_y);
        telemetry.addData("rot", gamepad1.right_trigger - gamepad1.left_trigger);
        telemetry.addData("lpb", left_back_power);
        telemetry.addData("lpf", left_front_power);
        telemetry.addData("rpb", right_back_power);
        telemetry.addData("rpf", right_front_power);

        //Set the velocities of each wheel motor
        left_front.setVelocity(left_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        left_back.setVelocity(left_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_front.setVelocity(right_front_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
        right_back.setVelocity(right_back_power * MAX_NUM_TICKS_MOVEMENT * MOVEMENT_RPM);
    }

    public void SetSpinnerForces() {
        if(!NoNullSpinner()) return;

        //Get the inputs
        boolean x = gamepad1.x;
        boolean b = gamepad1.b;

        //Set the spinner forces
        if(x) {
            telemetry.addData("x","x is active");
            spinner.setPower(-1);
        } else if(b) {
            spinner.setPower(1);
        } else {
            spinner.setPower(0);
        }
    }

    float craneRotationSpeedMultiplier = 0.002f;
    float craneArmJointMultiplier = 0.005f;
    float craneArmOrientationMultiplier = 0.005f;
    public void SetCraneIKTarget()
    {
        double xChange = 0, yChange = 0, orientationChange = 0;

        telemetry.addData("gamepad2left; gamepad2right", gamepad2.left_stick_y + "; " + gamepad2.right_stick_y);

        if(gamepad2.left_bumper)
            orientationChange = -1;
        else if(gamepad2.right_bumper)
            orientationChange = 1;

        //Source the ticks in the rotator
        float ticks = MAX_NUM_TICKS_ROTATOR;
        //what swath of the circle can the rotator rotate to? Half of the actual angle so 45 degrees represents a quarter of the circle
        float angleOfRotation = 45;
        //Half of the total range, positive and negative angle of rotation
        float rangeOfTicks = (angleOfRotation/360) * ticks;
        //Set the velocity
        double r = (gamepad2.right_trigger - gamepad2.left_trigger) * craneRotationSpeedMultiplier * ROTATOR_RPM * ticks;
        arm_rotator.setVelocity(r);
        //if(!(arm_rotator.getCurrentPosition() >= rangeOfTicks && r > 0) && !(arm_rotator.getCurrentPosition() <= -rangeOfTicks && r < 0)) {

        //}

        if((gamepad2.left_stick_y == 0 && gamepad2.right_stick_y == 0 && orientationChange == 0)) return;

        //Get the inputs for the arm
        xChange = gamepad2.left_stick_y * craneArmJointMultiplier;
        yChange = -gamepad2.right_stick_y * craneArmJointMultiplier;

        //Calculate the angles the arm must take
        double[] qs = math.IKArm(second_arm_joint.getPosition(), hand.getPosition(), xChange, yChange, orientationChange * craneArmOrientationMultiplier);

        if(qs == null || Double.isNaN(qs[0]) || Double.isNaN(qs[1]) || Double.isNaN(qs[2]))
            return;


        //Set the Servo Positions
        base_arm_joint.setPosition(math.Clamp(1 - (qs[0] / (3 / 2)), 0, 1));
        second_arm_joint.setPosition(math.Clamp(1 - (qs[1] / (3 / 2)), 0, 1));
        hand.setPosition(math.Clamp(1 - (qs[2] / (3 / 2)), 0, 1));
    }

    float gripperPosition = 0.4f;
    public void SetGripperForces() {
        //close the gripper
        if(gamepad2.a) {
            gripperPosition = 0.8f;
        }

        //open the gripper
        if(gamepad2.b) {
            gripperPosition = 0.4f;
        }

        gripper1.setPosition(gripperPosition);
    }
}

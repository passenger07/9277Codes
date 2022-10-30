package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ExplosivesUtils.ExplosiveTele;
import org.firstinspires.ftc.teamcode.ExplosivesUtils.PIDController;

@TeleOp(name = "FullTele")
public class FullTele extends ExplosiveTele {

    public PIDController armPID;


    @Override
    protected void initialize() {
        //robot.prepareForAuto();
    }

    boolean resetHeld = false;
    int resetNum = 0;

    @Override
    protected void looping() {
        final double deadban = 0.15;
        double scale = 0.5;
        
        double axial = -gamepad1.left_stick_y * scale;// neg
        double lateral = -gamepad1.left_stick_x * scale;// pos
        double yaw = gamepad1.right_stick_x * scale;// pos

        if (Math.abs(gamepad1.left_stick_y) > deadban || Math.abs(gamepad1.left_stick_x) > deadban || Math.abs(gamepad1.right_stick_x) > deadban) {
            robot.fright(axial + lateral + yaw);
            robot.bright(axial - lateral - yaw);
            robot.fleft(axial - lateral + yaw);
            robot.bleft(axial + lateral - yaw);
        } else {
            robot.stop();
        }
        if (Math.abs(gamepad2.left_stick_y) > 0.05 /*|| Math.abs(gamepad2.left_stick_x) > 0.2|| Math.abs(gamepad2.right_stick_x) > 0.2*/ ) {
         //  robot.armBase.setVelocity(500*gamepad2.left_stick_y);



              robot.targetArmBaseP += (int)(10.0*gamepad2.left_stick_y);




            robot.armBase.setTargetPosition(robot.targetArmBaseP);
            //robot.armBase.setPower(0.2);
        }


           



        if (Math.abs(gamepad2.right_stick_y) > 0.05 /* || Math.abs(gamepad2.right_stick_x) > 0.2|| Math.abs(gamepad2.right_stick_x) > 0.2*/) {
           // robot.armTop.setVelocity(500*gamepad2.right_stick_y);
             robot.targetArmTopP+=(int)(10.0*gamepad2.right_stick_y);
             robot.armTop.setTargetPosition(robot.targetArmTopP);
             //robot.armTop.setPower(0.2);

        }






        telemetry.addLine("fright:" + robot.fright.getVelocity());
        telemetry.addLine("bright:" + robot.bright.getVelocity());

        telemetry.addLine("fleft:" + robot.fleft.getVelocity());
        telemetry.addLine("bleft:" + robot.bleft.getVelocity());
        telemetry.addLine("armBase Setpoint: " + robot.targetArmBaseP);
        telemetry.addLine("armBase position "+robot.armBase.getCurrentPosition());//max -540
        telemetry.addLine("armTop Setpoint: " + robot.targetArmTopP);
        telemetry.addLine("armTop position "+robot.armTop.getCurrentPosition());           // 246
       // telemetry.addLine("Joint0: " + robot.arm.joint0.getCurrentPosition());
      //  telemetry.addLine("Joint1: " + robot.arm.joint1.getPosition());
        telemetry.update();



    }
}

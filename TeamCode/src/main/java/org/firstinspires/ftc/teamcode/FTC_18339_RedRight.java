package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FTC_18339_RedRight extends FTC_18339_AutonomousProtocol004 {
    @Override
    public void ChildCommandInitialization(){
        commands = new Command[] {
                new Command("IKTOPDROP", 0, true, 0),
                new Command("MOVE", -firstMoveC, true, 0),
                new Command("ROTATE", Math.toRadians(-rotationC), true, 0),
                new Command("MOVE", -secondMoveC, true, 0),
                new Command("GRIPPERDROP", 0, true, 0),
                new Command("ROTATE", Math.toRadians(rotationC), true, 0),
                new Command("MOVE", thirdMoveW, true, 0),
                new Command("ROTATE", Math.toRadians(-70), true, 0),
                new Command("MOVE", fourthMoveW, true, 0),
                new Command("IKFORSTART", 0, true, 0)
        };
    }
}

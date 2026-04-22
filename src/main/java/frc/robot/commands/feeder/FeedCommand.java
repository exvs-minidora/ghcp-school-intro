package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;

/** Feeder 単独でフィード方向に動かすコマンド。 */
public class FeedCommand extends Command {

    private final FeederSubsystem feeder;

    public FeedCommand(FeederSubsystem feeder) {
        this.feeder = feeder;
        addRequirements(feeder);
    }

    @Override
    public void execute() {
        feeder.feed();
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

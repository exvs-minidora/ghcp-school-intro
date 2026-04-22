package frc.robot.commands.extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extension.ExtensionSubsystem;

/** Extension を格納位置まで戻すコマンド。目標到達で完了する。 */
public class RetractCommand extends Command {

    private final ExtensionSubsystem extension;

    public RetractCommand(ExtensionSubsystem extension) {
        this.extension = extension;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        extension.retract();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            extension.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return extension.isAtTarget();
    }
}

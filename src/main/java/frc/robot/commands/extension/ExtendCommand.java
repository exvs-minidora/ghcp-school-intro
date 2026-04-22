package frc.robot.commands.extension;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.extension.ExtensionSubsystem;

/** Extension をフル展開位置まで動かすコマンド。目標到達で完了する。 */
public class ExtendCommand extends Command {

    private final ExtensionSubsystem extension;

    public ExtendCommand(ExtensionSubsystem extension) {
        this.extension = extension;
        addRequirements(extension);
    }

    @Override
    public void initialize() {
        extension.extend();
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

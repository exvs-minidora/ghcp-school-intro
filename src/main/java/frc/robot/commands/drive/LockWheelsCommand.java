package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DrivebaseSubsystem;

/**
 * 全ホイールを X フォーメーションにしてロボットを制動するコマンド。
 */
public class LockWheelsCommand extends Command {

    private final DrivebaseSubsystem drivetrain;

    public LockWheelsCommand(DrivebaseSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.lockWheels();
    }

    @Override
    public boolean isFinished() {
        return false; // トリガー保持中は継続する
    }
}

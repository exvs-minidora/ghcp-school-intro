package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * Shooter をスピンアップし、準備完了後に Feeder でノートを送り込む射出コマンド。
 *
 * <p>ボタン保持中は継続し、離すと両モータを停止する。
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final FeederSubsystem  feeder;

    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder) {
        this.shooter = shooter;
        this.feeder  = feeder;
        addRequirements(shooter, feeder);
    }

    @Override
    public void initialize() {
        shooter.spinUp();
    }

    @Override
    public void execute() {
        if (shooter.isAtVelocity()) {
            feeder.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

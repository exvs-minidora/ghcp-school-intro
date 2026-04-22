package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

/**
 * 手動フォールバック用射出コマンド (Hood なし)。
 * Shooter 収束後 Feeder を起動し、{@link SpindexerConstants#SPINDEXER_DELAY_S}
 * 秒後に Spindexer を段階起動する。
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem   shooter;
    private final FeederSubsystem    feeder;
    private final SpindexerSubsystem spindexer;

    private final Timer spindexerTimer = new Timer();
    private boolean     feederStarted  = false;

    public ShootCommand(ShooterSubsystem shooter, FeederSubsystem feeder,
                        SpindexerSubsystem spindexer) {
        this.shooter   = shooter;
        this.feeder    = feeder;
        this.spindexer = spindexer;
        addRequirements(shooter, feeder, spindexer);
    }

    @Override
    public void initialize() {
        shooter.spinUp();
        feeder.stop();
        spindexer.stop();
        feederStarted = false;
        spindexerTimer.stop();
        spindexerTimer.reset();
    }

    @Override
    public void execute() {
        if (shooter.isAtVelocity()) {
            feeder.feed();
            if (!feederStarted) {
                feederStarted = true;
                spindexerTimer.restart();
            }
            if (spindexerTimer.hasElapsed(SpindexerConstants.SPINDEXER_DELAY_S)) {
                spindexer.spin();
            }
        } else {
            feeder.stop();
            spindexer.stop();
            feederStarted = false;
            spindexerTimer.stop();
            spindexerTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop();
        spindexer.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

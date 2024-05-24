package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalSource;
import java.util.function.BiConsumer;

public class WaitUntilInterrupt extends Command {
	private boolean hasFinished = false;
	private final AsynchronousInterrupt interrupt;
	
	/**
	 * Creates a new Asynchronous Interrupt and finishes when the interrupt is triggered.
	 *
	 * @param source
	 *                The digital Source to use.
	 * @param callback
	 *                The callback to call on interrupt.
	 * @param risingEdge
	 *                Trigger on the rising edge.
	 * @param fallingEdge
	 *                Trigger on the falling edge.
	 */
	public WaitUntilInterrupt(DigitalSource source, BiConsumer<Boolean, Boolean> callback, boolean risingEdge, boolean fallingEdge) {
		interrupt = new AsynchronousInterrupt(source, (rising, falling) -> {
			callback.accept(rising, falling);
			hasFinished = true;
		});
		interrupt.setInterruptEdges(risingEdge, fallingEdge);
	}
	
	@Override
	public void initialize() {
		hasFinished = false;
		interrupt.enable();
	}
	
	@Override
	public void end(boolean interrupted) {
		interrupt.disable();
	}
	
	@Override
	public boolean isFinished() {
		return hasFinished;
	}
}

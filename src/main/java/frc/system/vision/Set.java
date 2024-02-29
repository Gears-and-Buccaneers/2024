package frc.system.vision;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import frc.system.Vision;

/** Aggregates and averages weighted vision data from a set of sources. */
public class Set implements Vision {
	final List<Vision> sources;

	/** Creates a new instances with the provided sources. */
	public Set(Vision... sources) {
		this.sources = Arrays.asList(sources);
	}

	@Override
	public void register(Consumer<Measurement> measurement) {

	}
}

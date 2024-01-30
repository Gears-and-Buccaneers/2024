package frc.system.vision;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import frc.system.Vision;

public class Set implements Vision {
	final List<Vision> sources;

	public Set(Vision... sources) {
		this.sources = Arrays.asList(sources);
	}

	@Override
	public void with(Consumer<Measurement> measurement) {

	}
}

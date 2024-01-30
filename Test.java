import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public class Test {
	public static void main(String[] args) {
		List<Double> x = new ArrayList<>();

		x.add(23.0);
		x.add(349.2);
		x.add(92.7);

		Stream<Double> stream = x.stream();

		Double[] list1 = stream.map(i -> i + 11).toArray(Double[]::new);
		Double[] list2 = x.stream().map(i -> i * 2).toArray(Double[]::new);

		for (int i = 0; i < list1.length; i++)
			System.out.println("list1[" + i + "] = " + list1[i]);

		for (int i = 0; i < list2.length; i++)
			System.out.println("list2[" + i + "] = " + list2[i]);
	}
}

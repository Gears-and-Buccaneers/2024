public class Test {
	static abstract class Test1 {
		final double x;

		public Test1(int n) {
			x = n;
		}
	}

	static class Test2 extends Test1 {
		final double z = x + 3;

		public Test2() {
			super(6);
		}
	}

	public static void main(String[] args) {
		Test2 x = new Test2();
		System.out.println(x.z);
	}

	// driver.lX
	// driver.lX()
}

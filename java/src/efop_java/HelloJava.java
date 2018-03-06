package efop_java;

public class HelloJava {

	private String name;
	private double number;
	
	public HelloJava() {
		setName(" cicca ");
		setNumber(-1);
		System.out.println("The java classes send their regards");
	}

	public double getNumber() {
		return number;
	}

	public void setNumber(double number) {
		this.number = number;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}
	
	
	
	
}

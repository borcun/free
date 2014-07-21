import java.util.Scanner;

public class BirdTest {

	public static void main(String[] args) 
	{
		Scanner input = new Scanner(System.in);
		String name, environment;
		int age;
		
		Bird bird = new Bird();
		Bird kanarya = new Bird("Kanarya");
		Bird papagan = new Bird("Papagan", "Ekvator");
		
		System.out.print("Lutfen kusun adini giriniz : ");
		name = input.nextLine();
		
		System.out.print("\nLutfen kusun yasadigi ortami giriniz : ");
		environment = input.nextLine();

		System.out.print("\nLutfen kusun yasini giriniz : ");
		age = input.nextInt();
		
		bird.setName(name);
		bird.setEnvironment(environment);
		bird.setAge(age);

		// kanarya
		System.out.printf("\nLutfen %s'in yasadigi ortami giriniz : ", kanarya.getName());
		environment = input.nextLine();
		environment = input.nextLine();
		
		System.out.printf("\nLutfen %s yasini giriniz : ", kanarya.getName());
		age = input.nextInt();
		
		kanarya.setEnvironment(environment);
		kanarya.setAge(age);
		
		// papagan
		System.out.printf("\nLutfen %s yasini giriniz : ", papagan.getName());
		age = input.nextInt();
		
		papagan.setAge(age);
		
		bird.printBirdAttributes();
		kanarya.printBirdAttributes();
		papagan.printBirdAttributes();
	}

}

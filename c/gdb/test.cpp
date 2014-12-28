#include <iostream>

void my_print()
{
	int x;

	std::cout << "I am printer" << std::endl;
	std::cin >> x;
	std::cout << "You entered " << x << std::endl;

	return;
}

int main(int argc, char **argv)
{
	int i = argc;

	std::cout << "argc: " << argc << std::endl;
	for(int i=0 ; i < argc ; ++i)
		std::cout << "argv[" << i << "] : " << argv[i] << std::endl;

	std::cout << "Enter a number: ";
	std::cin >> i;

	switch(i) {
	case 0:
		std::cout << "You entered 0" << std::endl;
		break;
	case 1:
		std::cout << "You entered 1" << std::endl;
		break;
	default:
		std::cout << "You entered undetected number :)" << std::endl;
		break;
	}

	std::cout << "Program terminates successfully" << std::endl;

	return 0;
}

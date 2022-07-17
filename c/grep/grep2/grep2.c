#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_CHAR 255 

// function that shows usage of application
void usage(const char *argv);
// function that finds and prints word in file
void find(const char *file, const char *word);

int main(int argc, char **argv)
{
	int i;

  if(argc < 3) {
    usage(argv[0]);
    return -1;
  }

	for(i = 1 ; i < argc - 1 ; ++i)
		find(argv[i], argv[argc - 1]);

  return 0;
}

// usage function
void usage(const char *argv)
{
  printf("usage: %s <first file> <second file> ... <word>", argv);
  return;
}

// find function
void find(const char *path, const char *word)
{
	FILE *fptr;
	char line[MAX_CHAR] = {'\0'};
	int count = 0;
	int isFound = 0;

	printf("File  : %s\n", path);
	printf("State : ");

	if(NULL == (fptr = fopen(path, "r"))) {
		printf("NOT OPENED\n");
		return;
	}

	printf("OPENED\n");
	printf("Lines : ");

	// search
	while(NULL != fgets(line, MAX_CHAR, fptr)) {
		if(NULL != strstr(line, word)) {
			printf("%d, ", count);
			isFound = 1;
		}

		memset(line, '\0', MAX_CHAR);
		++count;
	}

	if(0 == isFound)
		printf("NOT FOUND");

	printf("\n\n");
	fclose(fptr);

	return;
}
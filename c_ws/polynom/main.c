#include <stdio.h>
#include <string.h>

#define MAX_CHAR 255
#define CMD_SIZE 4
#define ADD "ADD"
#define SUB "SUB"

// polynom structure
typedef struct Polynom
{
  int x2;
  int x1;
  int c;
} Polynom;

// function that parse polynom and retrieves its coefficients
void parsePolynom(Polynom *p, char *pline);

// main function
int main(int argc, char **argv)
{
  FILE *fptr;
  char line[MAX_CHAR] = {'\0'};
  char command[CMD_SIZE] = {'\0'};
  char polynom1[MAX_CHAR] = {'\0'};
  char polynom2[MAX_CHAR] = {'\0'};
  Polynom p1 = {0, 0, 0};
  Polynom p2 = {0, 0, 0};

  // open file
  if(NULL == (fptr = fopen(argv[1], "r"))) {
    fprintf(stderr, "FILE NOT OPENED");
    return -1;
  }

  if(NULL == fgets(line, MAX_CHAR, fptr)) {
    fprintf(stderr, "fgets error");
    return -1;
  }

  strcpy(command, strtok(line, " "));
  strcpy(polynom1, strtok(NULL, " "));
  strcpy(polynom2, strtok(NULL, " "));

  printf("Command: %s\n", command);
  printf("Polynom1: %s\n", polynom1);
  printf("Polynom2: %s\n", polynom2);


  if(!strcmp(ADD, command)) {
    parsePolynom(&p1, polynom1);
    parsePolynom(&p2, polynom2);
    
    printf("Polynom I : %dx^2 + %dx^1 + %d\n", p1.x2, p1.x1, p1.c);
    printf("Polynom II: %dx^2 + %dx^1 + %d\n", p2.x2, p2.x1, p2.c);
    printf("Total Polynom: %dx^2 + %dx^1 + %d\n", p1.x2 + p2.x2, p1.x1 + p2.x1 , p1.c + p2.c);
  }
  else if(!strcmp(SUB, command)) {
    parsePolynom(&p1, polynom1);
    parsePolynom(&p2, polynom2);
    
    printf("Polynom I : %dx^2 + %dx^1 + %d\n", p1.x2, p1.x1, p1.c);
    printf("Polynom II: %dx^2 + %dx^1 + %d\n", p2.x2, p2.x1, p2.c);
    printf("Total Polynom: %dx^2 + %dx^1 + %d\n", p1.x2 - p2.x2, p1.x1 - p2.x1 , p1.c - p2.c);
  }
  else {
    fprintf(stderr, "invalid command\n");
  }

  // close file
  fclose(fptr);

  return 0;
}

void parsePolynom(Polynom *p, char *line)
{
  int number = 0;
  int i, j, k;
  char term[3][8] = {{'\0'}};
  char *temp;

  if(NULL == line) {
    fprintf(stderr, "invalid data\n");
    return;
  }

  temp = strtok(line, "+-");
  if(NULL != temp) {
    strcpy(term[0], temp);
    for(i = 1 ; NULL != (temp = strtok(NULL, "+-")) ; ++i)
      strcpy(term[i], temp);
  }

  for(j=0 ; j < i-1 ; ++j) {
    for(k=0 ; k < strlen(term[j]) ; ++k) {
      if(term[j][k] == 'x') {
	k += 2;
	if(term[j][k] == '2') {
	  p->x2 = number;
	  number = 0;
	}
	else if(term[j][k] == '1') {
	  p->x1 = number;
	  number = 0;
	}
      }
      else
	number += (term[j][k] - 48);
    }
  }

  sscanf(term[i-1], "%d", &p->c);

  return;
}

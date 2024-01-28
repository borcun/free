#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef enum {
  NONE_COLOR,
  BLUE_HAT,
  GREEN_HAT
} Hat;

typedef enum {
  EVEN,
  ODD
} Parity;

typedef struct {
  Parity blue;
  Parity green;
} HatParity;

// function that estimate lay off list
void layoff(Hat *hat_list, const unsigned int size, const Hat former_hat, const HatParity former_parity);
// function that fills hat list randomly
void fill(Hat *hat_list, const unsigned int size);
// function that display hat list
void display(Hat *hat_list, const unsigned int size);
// function that gets parity values of blue and green hat before position
HatParity getParity(Hat *hat_list, const unsigned int pos);
// function that prints usage of program
void usage(void);

int main(int argc, char **argv) {
  Hat *hat_list = NULL;
  HatParity parity;
  int total_hat = 0;
  
  if (2 != argc) {
    usage();
    return -1;
  }

  total_hat = atoi(argv[1]);
  
  if (0 != total_hat % 2 || total_hat > 100) {
    usage();
    return -1;
  }

  hat_list = (Hat *) malloc(total_hat * sizeof(Hat));

  if (NULL == hat_list) {
    printf("%s\n", "Could not allocate memory for hat list");
    return -1;
  }

  printf("%s\n\n",
	 "Note: The employees who have different hat colors into\n" \
	 "the initial list and estimation list will be laid off.\n" \
	 "[H] refers to head of the list, [T] to tail of the list.");
  
  fill(hat_list, total_hat);

  printf("%s\n", "Initial List");
  display(hat_list, total_hat);

  printf("\n%s\n%s", "Estimation List", "[H] ");
  layoff(hat_list, total_hat, NONE_COLOR, parity);
  printf("%s\n", "[T]");

  free(hat_list);
  
  return 0;
}

void layoff(Hat *hat_list, const unsigned int size, const Hat former_hat, const HatParity former_parity) {
  if (1 > size) {
    printf("%s\n", "Invalid size for the hat list");
    return;
  }

  // if there is one hat, odd one into the former parity determines current hat color directly
  // thus, we do not need extra effort to estimate current hat color of the last employee
  if (1 == size) {
    printf("%s ", (ODD == former_parity.blue) ? "B" : "G");
    return;
  }

  HatParity curr_parity = getParity(hat_list, size - 1);
  Hat curr_hat = NONE_COLOR;
  
  // the first estimation must be done randomly, because we do not have anything to estimate it
  if (NONE_COLOR == former_hat) {
    // we select former hat according to odd number hat count except the last employee,
    // it is just arbitrary selection, and strongly assumed that the entire hat list include
    // even number hats, otherwise the algorithm fails
    curr_hat = (ODD == curr_parity.blue) ? BLUE_HAT : GREEN_HAT;
    
    layoff(hat_list, size - 1, curr_hat, curr_parity);
    printf("%s ", curr_hat == BLUE_HAT ? "B" : "G");
  }
  else {
    // if blue parity of former and current hats are same, it means that current hat is green
    // because if parity of any color does not change from former one to current one, the color
    // whose parity changes is color of the current hat
    if (former_parity.blue == curr_parity.blue) {
      curr_hat = GREEN_HAT;
    }
    // otherwise, current hat is blue
    else {
      curr_hat = BLUE_HAT;
    }
    
    layoff(hat_list, size - 1, curr_hat, curr_parity);
    printf("%s ", curr_hat == BLUE_HAT ? "B" : "G");
  }
}

void fill(Hat *hat_list, const unsigned int size) {
  srand(time(NULL));

  for (unsigned int i = 0; i < size; ++i) {
    hat_list[i] = (rand() % 2) ? BLUE_HAT : GREEN_HAT;
  }

  return;
}

void display(Hat *hat_list, const unsigned int size) {
  printf("%s ", "[H]");
  
  for (unsigned int i = 0; i < size; ++i) {
    if (NONE_COLOR != hat_list[i]) {
      printf("%s ", BLUE_HAT == hat_list[i] ? "B" : "G");
    }
    else {
      printf("%s ", "X");
    }
  }

  printf("%s\n", "[T]");

  return;
}

HatParity getParity(Hat *hat_list, const unsigned int pos) {
  HatParity parity;
  int blue_hat = 0;
  int green_hat = 0;

  for (unsigned int i = 0; i < pos; ++i) {
    hat_list[i] == BLUE_HAT ? ++blue_hat : ++green_hat;
  }

  parity.blue = (0 == blue_hat % 2) ? EVEN : ODD;
  parity.green = (0 == green_hat % 2) ? EVEN : ODD;
  
  return parity;
}

void usage(void) {
  printf("%s\n", "The program solves a layoff problem, or hat puzzle");
  printf("%s\n", "There are some employees lined up on a line from back to front,\n" \
	 "and the employer puts blue or green hat on their head randomly.\n" \
	 "The purpose of the program is finding an optimization algorithm\n" \
	 "to lay off minimum employees.");
  printf(" %s\n", "usage: ./layoff <employee count in odd number>");
  
  return;
}

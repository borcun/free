Problem

FA figure illustrating the propagation of Fibonacci's rabbits if they die after three months.
Recall the definition of the Fibonacci numbers from “Rabbits and Recurrence Relations”, which 
followed the recurrence relation Fn=Fn−1+Fn−2 and assumed that each pair of rabbits reaches 
maturity in one month and produces a single pair of offspring (one male, one female) each 
subsequent month.

Our aim is to somehow modify this recurrence relation to achieve a dynamic programming solution 
in the case that all rabbits die out after a fixed number of months. See Figure 4 for a depiction 
of a rabbit tree in which rabbits live for three months (meaning that they reproduce only twice 
before dying).

Given: Positive integers n≤100 and m≤20.
Return: The total number of pairs of rabbits that will remain after the n-th month if all rabbits 
live for m months.

Sample Dataset
6 3

Sample Output
4
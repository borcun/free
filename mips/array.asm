.data
A: .word 2, 13 , 1, 4, 8, 20 , 5, 6, 9, 14
S: .word 10
K: .word 10

.text
.globl main

main:
	la $s0, A        # load base address of array A
	lw $s1, S        # load size of array A
	lw $s2, K        # load threshold value
	addi $t0, $0, 0  # index that is used when travelling array A
	addi $t1, $0, 0  # counter used to count elements that are less than threshold K
	
	while:
		# if index < S (currently 10), break the loop (means jump to done label)
	  slt $t2, $t0, $s1
		bne $t2, 1, done
		
		# calculate 4 bytes width offset at each iteration in order to travel on array A
		sll $t3, $t0, 2
		# get current address adding the offset value onto base address of array A
		add $t3, $t3, $s0
		# read value at the current address
		lw $t2, 0($t3)
		
		if:
		  # first condition of if statement, and if it fails, jump to end of if block directly similar to shortcut
		  # if value < K (10), then the first condition is OK, and continue processing of the second condition
			slt $t3, $t2, $s2
			bne $t3, 1, fi
			
			# AND
			
			# second condition of if statement
			# even number checking via controlling last bit of value whether it is presence
			andi $t3, $t2, 0x00000001
			bne $t3, 0, fi
			
			# operation inside if block, increasing counter if two conditions are OK
			addi $t1, $t1, 1
		fi:
		
		# increase index one, which is used to travel array A
		addi $t0, $t0, 1
		j while	
	done:
	
	# syscall 10 is termination execution like exit function in C
	li $v0, 0x0A
	syscall

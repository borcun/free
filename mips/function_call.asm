.text

# no need prologue and epilogue code blocks for compute_alpha and compute_beta functions,
# because they are top functions in function call stack, so no more update for ra register

compute_alpha:
	# Computes 4*a - b
	sll $t0, $a0, 2
	sub $v0, $t0, $a1 
	jr $ra

compute_beta:
	# Computes 2*a + 3*b
	sll $t0, $a0, 1
	sll $t1, $a1, 1
	add $t1, $t1, $a1
	add $v0, $t0, $t1
	jr $ra

process_data:
  # prologue, just store return address because no more persistent register here
	addi $sp, $sp, -4
	sw $ra, 0($sp)

	# if a % 2 == 0 call computer_alpha, otherwise call computer_beta
	andi $t0, $a0, 1
	bne $t0, 0, call_compute_beta
	
	call_computer_alpha:
	jal compute_alpha
	j end_of_call
	call_compute_beta:
	jal compute_beta
	
	end_of_call:
	
	# epilogue, get return address from stack
	lw $ra, 0($sp)
	addi $sp, $sp, 4
	jr $ra
	
.globl main
main:
	addi $a0, $0, 5  # parameter a to process_data function
	addi $a1, $0, 4  # parameter b to process_data function
	
	if:
		slt $t0, $a0, $a1
		beq $t0, 0, else
		# equivalent to 4 * (a + b)
		add $t0, $a0, $a1
		sll $t0, $t0, 2
	  # put result into v0 to be returned 
		addi $v0, $t0, 0
		j fi
	else:
		# no need extra operation to put result into v0, it's already put into there by process_data
		jal process_data
	fi:
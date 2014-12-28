#!/bin/bash
for (( i = 1; i <= 8; i++ )) ### Outer for loop ###
do
   for (( j = 1 ; j <= 8; j++ )) ### Inner for loop ###
   do
        total=$(( $i + $j))   # total 
        tmp=$(( $total % 2))  # modulus
        # Find out odd and even number and change the color 
        # alternating colors using odd and even number logic  
        if [ $tmp -eq 0 ]; 
        then
            echo -e -n "\033[47m  "
        else
            echo -e -n "\033[40m  "
        fi
  done
 echo "" #### print the new line ###
done
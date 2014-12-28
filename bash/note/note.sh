#!/usr/bin/bash
# Script to print currently logged in users information, and current date & time.
clear
echo "Hello $USER"
echo -e "Today is \c ";date
echo -e "Number of user login : \c" ; who | wc -l
echo "Calendar"
cal
exit 0

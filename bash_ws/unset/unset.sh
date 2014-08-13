#1/usr/bin/bash

#unset command delete content of variable in execution time
var="content is not empty"
echo "content: '${var}'"
unset var
echo "content: '${var}'"

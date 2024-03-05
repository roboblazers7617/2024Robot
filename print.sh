#!/bin/sh
output=$(grep System.out.print -n -r --include "*.java" --exclude ./src/main/java/frc/robot/util/Alert.java)
empty=""
# echo $output
if test "$output" = "" 
then
	echo "No System.out.print found"
	exit 0
else
	echo "System.out.print found"
	printf $output
	exit 1
fi

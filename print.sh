#!/bin/sh
output=$(grep -v -E "(?:/\\*(?:[^*]|(?:\\*+[^*/]))*\\*+/)|(?://.*)" -r --include "*.java" --exclude ./src/main/java/frc/robot/util/Alert.java | grep System.out.print)
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

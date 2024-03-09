start=$(date +%s)
./gradlew simulateJava &
sleep 15
exit $(kill "$!")
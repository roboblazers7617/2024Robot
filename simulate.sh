start=$(date +%s)
./gradlew simulateJava &
sleep 10
exit $(kill "$!")
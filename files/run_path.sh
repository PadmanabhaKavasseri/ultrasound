
adb wait-for-device
adb shell tdk-chx01-fw -p ch101_v41b.bin
adb shell tdk-chx01-fw -p ch201_v10a.bin
adb shell tdk-chx01-get-data-app -d 10 -s 120 -f 5 -l /usr/chirp.csv
adb pull /usr/chirp.csv ./

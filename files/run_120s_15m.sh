
adb wait-for-device
adb shell tdk-chx01-get-data-app -d 900  -s 120 -f 1 -l /usr/chirp.csv
adb pull /usr/chirp.csv ./

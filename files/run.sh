
adb wait-for-device
adb shell tdk-chx01-get-data-app -d 5  -s 200 -f 5 -l /usr/chirp.csv
adb pull /usr/chirp.csv ./

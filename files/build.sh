
# adb wait-for-device
# adb push tdk-chx01-get-data.c /usr/
# adb shell "gcc /usr/tdk-chx01-get-data.c -o /usr/local/bin/tdk-chx01-get-data-app -L/usr/bin -lInvnAlgoRangeFinder"

gcc tdk-chx01-get-data.c -o tdk-chx01-get-data-app -L. -lInvnAlgoRangeFinder

#adb wait-for-device
#adb push tdk-chx01-get-data.c /usr/
gcc tdk-chx01-get-data.c -o chirp -L. -lInvnAlgoRangeFinder -lm

gcc -shared -fPIC tdk-chx01-get-data.c -o libchirp.so -L. -lInvnAlgoRangeFinder -lm


gcc tdk-chx01-get-data.c -o tdk-chx01-get-data-app -L. -lInvnAlgoRangeFinder -lInvnAlgoFloorTypeFxp -lInvnAlgoCliffDetection -lInvnAlgoObstaclePosition -lm

gcc -shared -fPIC tdk-chx01-get-data.c -o libtdk-chx01-get-data.so -L. -lInvnAlgoRangeFinder -lInvnAlgoFloorTypeFxp -lInvnAlgoCliffDetection -lInvnAlgoObstaclePosition -lm

cp libtdk-chx01-get-data.so /usr/lib/.

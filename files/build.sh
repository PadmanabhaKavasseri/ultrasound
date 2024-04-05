
adb wait-for-device
adb shell mkdir /usr/share/tdk/
adb push tdk-chx01-get-data.c /usr/

adb push ./libInvnAlgoRangeFinder.a /usr/bin/
adb push ./libInvnAlgoFloorTypeFxp.a /usr/bin/
adb push ./libInvnAlgoCliffDetection.a /usr/bin/
adb push ./libInvnAlgoObstaclePosition.a /usr/bin/

adb push ./invn_algo_rangefinder.h /usr/
adb push ./invn_algo_cliff_detection.h /usr/
adb push ./invn_algo_floor_type_fxp.h /usr/
adb push ./invn_algo_obstacleposition.h /usr/

adb push ./invn/ /usr/
adb push firmware/* /usr/share/tdk/
adb shell "gcc /usr/tdk-chx01-get-data.c -o /usr/local/bin/tdk-chx01-get-data-app -L/usr/bin -lInvnAlgoRangeFinder -lInvnAlgoFloorTypeFxp -lInvnAlgoCliffDetection -lInvnAlgoObstaclePosition -lm"


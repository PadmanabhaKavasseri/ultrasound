rm -rf tdk-chx01-get-data-app

gcc tdk-chx01-get-data.c -g -o tdk-chx01-get-data-app -L. -lInvnAlgoRangeFinder -lInvnAlgoFloorTypeFxp -lInvnAlgoCliffDetection -lInvnAlgoObstaclePosition -lm

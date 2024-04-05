
adb wait-for-device
adb shell tdk-chx01-fw -n 2
adb shell tdk-chx01-fw -n 5
for (( c=1; c<=20; c++ ))
do
adb shell tdk-chx01-get-data-app -d 10  -s 120 -f 5 -l /usr/chirp.csv -n
echo $c
done
adb pull /usr/chirp.csv ./

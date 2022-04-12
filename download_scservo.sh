TARGET="https://akizukidenshi.com/download/ds/feetech/SCServo.zip"
echo "start downloading SCServo.zip from ${TARGET}"
curl -OL ${TARGET}
unzip SCServo.zip
mv SCServo/*.h include/
mv SCServo/*cpp src/
rm SCServo.zip
rm -rf SCServo
## Port 고정 세팅에 관해

<br>

### First Step
```
lsusb
```
-> 다음 사항 확인<br>
Bus 001 Device 007: ID **0403:6014** Future Technology Devices International, Ltd FT232H Single HS USB-UART/FIFO IC<br>
**0403, 6014** 확인<br><br>


### Second Step
```
udevadm info -a /dev/ttyUSB0 | grep '{serial}'
```
-> 다음 사항 확인<br>
ATTRS{serial}=="FT4TFNO6"<br>
**FT4TFNO6** 확인
<br><br>

### Third Step
.rules 파일 작성<br>
```
cd /etc/udev/rules.d
```
```
sudo vim 99-imu.rules
```

<br>

`.rules`파일에 다음 사항 작성

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", ATTRS{serial}=="FT4TFNO6", SYMLINK+="IMU0"
```

**SYMLINK** -> 원하는 포트 이름 작성<br><br>

### Last Step
```
sudo service udev restart
```
```
sudo reboot
```
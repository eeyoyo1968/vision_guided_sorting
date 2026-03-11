### 6. Start SOCAT Gripper Bridge (background)
echo "[5/6] Starting gripper socat bridge..."
bash -c "
while true; do
  echo 'Reviving Bridge...'
  sudo killall -9 socat 2>/dev/null
  socat pty,link=/tmp/ttyUR,raw,echo=0,waitslave tcp:192.168.2.2:54321,nodelay
  sleep 1
done
" > /tmp/socat.log 2>&1 &

### Ensure permissions
sleep 2
sudo chmod 666 /tmp/ttyUR || true


### 7. Start Gripper Server
echo "[6/6] Starting gripper server..."
python3 ~/humble_ws/src/my_ur_description/scripts/robotiq_gripper_server.py \
  > /tmp/gripper_server.log 2>&1 &

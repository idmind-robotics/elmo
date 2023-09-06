#! /bin/bash

export DISPLAY=:0
until $(curl --output /dev/null --silent --head --fail http://localhost:8000); do
  sleep 1
done
/usr/bin/chromium --kiosk --app=http://localhost:8000
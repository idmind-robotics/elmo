#!/bin/sh


BASE_GPIO_PATH=/sys/class/gpio
SHUTDOWN=4

ON="1"
OFF="0"
# Utility function to export a pin if not already exported
exportPin()
{
  if [ ! -e $BASE_GPIO_PATH/gpio$1 ]; then
    echo "$1" > $BASE_GPIO_PATH/export
  fi
}

# Utility function to set a pin as an output
setOutput()
{
  echo "out" > $BASE_GPIO_PATH/gpio$1/direction
}

# Utility function to change state of a light
setPinState()
{
  echo $2 > $BASE_GPIO_PATH/gpio$1/value
}

turnOff()
{
  echo 'turning off'
  setPinState $SHUTDOWN $OFF
}

exportPin $SHUTDOWN
setOutput $SHUTDOWN
turnOff

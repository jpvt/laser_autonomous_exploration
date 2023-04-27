#!/bin/bash

# Laser L1BR Autonomous Exploration
# João Vasconcelos - jpvteixeira99@gmail.com
# First Version: 2023-04-27

echo -e ""
echo -e "⚙️  Setting up Laser L1BR Autonomous Exploration Environment"
echo -e ""
echo -e "➡️  This build takes ~ 5 minutes on a 300Mbps connection (first run)"
echo -e "➡️  and stores ~ 4GB of data on your disk."
echo -e ""

echo -e ""
echo -e "⚙️  Starting environment with docker-compose..."
echo -e ""
xhost +local:docker && sudo docker compose up -d

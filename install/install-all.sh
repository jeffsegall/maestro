#!/bin/bash
set -e
echo "Comprehensive Install Script for Maestro"
echo "Version 1.0"
echo ""

echo "Installing ROS-Orocos-Maestro..."
sudo bash install-fuerte.sh -y
echo "Installing Hubo-ACH..."
sudo bash install-hubo-ach.sh
echo "Installing OpenHUBO..."
bash install-openHubo.sh

echo "Install Complete. Exiting..."

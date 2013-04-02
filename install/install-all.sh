#!/bin/bash
set -e
echo "Comprehensive Install Script for Maestro"
echo "Version 1.0"
echo ""

echo "Installing ROS-Orocos-Maestro..."
sudo sh install-fuerte.sh
echo "Installing Hubo-ACH..."
sudo sh install-hubo-ach.sh
echo "Installing OpenHUBO..."
sh install-openHubo.sh

echo "Install Complete. Exiting..."

#!/bin/bash
echo "Comprehensive Install Script for Maestro"
echo "Version 1.0"
echo ""

echo "Installing ROS-Orocos-Maestro..."
sh -c ' install-fuerte.sh'
echo "Installing Hubo-ACH..."
sh -c ' install-hubo-ach.sh'
echo "Installing OpenHUBO..."
sh -c ' install-openHubo.sh'

echo "Install Complete. Exiting..."

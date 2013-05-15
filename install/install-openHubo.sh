set -e
echo "OpenHUBO Maestro installation Script"
echo "Version 1.0"
echo ""

if [[ $# > 0 && $1 == "-y" ]]; then
	QUIET=true
fi

if [[ -z "$QUIET" ]]; then
	read -p "Please enter installation directory (No trailing '/' Please): " installDir
else
	installDir=~/
fi

if [[ ! -d "$installDir" ]]; then
	mkdir $installDir
	if [[ ! -d "$installDir" ]]; then
		echo "Failed to create install directory."
		exit 1
	fi
fi

cd "$installDir"

git clone https://github.com/daslrobotics/openHubo

cd openHubo

sh -c './setup'

if [[ -z $(grep "source $installDir/openHubo/env.sh" ~/.bashrc)  ]]; then
	if [[ -z "$QUIET" ]]; then
		echo ""
		echo "Would you like to add a source line to your bashrc file?"
		select yn in "Yes" "No"; do
        		case $yn in
                	Yes )
				echo "Adding source command to bashrc..."; 
				echo "source $installDir/env.sh" >> ~/.bashrc; 
				break;;
                	No ) 
				echo "Skipping modification of bashrc..."; 
				break;;
        		esac
		done
	else
		echo "source $installDir/env.sh" >> ~/.bashrc;
	fi
fi
source $installDir/openHubo/env.sh


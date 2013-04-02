echo "OpenHUBO Maestro installation Script"
echo "Version 1.0"
echo ""

installDir=~/openHubo
mkdir $installDir
cd $installDir/../

git clone https://github.com/daslrobotics/openHubo

cd openHubo

sh -c './setup'

echo ""
echo "Would you like to add a source line to your bashrc file?"
select yn in "Yes" "No"; do
        case $yn in
                Yes ) echo "Adding source command to bashrc..."; echo "source $installDir/env.sh" >> ~/.bashrc; break;;
                No ) echo "Skipping modification of bashrc..."; break;;
        esac
done

source $installDir/env.sh


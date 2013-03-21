installDir=~/git/openHubo
mkdir $installDir
cd $installDir/../

git clone https://github.com/daslrobotics/openHubo

cd openHubo

sh -c './setup'

echo "source $installDir/env.sh" >> ~/.bashrc

source $installDir/env.sh


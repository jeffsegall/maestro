apt-get purge mercurial ros-fuerte-desktop-full libreadline-dev omniorb omniidl omniorb-nameserver libomniorb4-1 libomniorb4-dev libomnithread3-dev libomnithread3c2 gccxml antlr libantlr-dev libxslt1-dev liblua5.1-0-dev ruby1.8-dev libruby1.8 rubygems1.8 libboost-dev
apt-get autoremove

sh -c ' hubo-ach-clean-all.sh'
apt-get purge libach1 libach-dev ach-utils hubo-ach hubo-ach-dev

rm -rf /opt/ros
 

#!/bin/bash
#
# This script is intended to crawl through the /include and /src directories of # the current project looking for source files to add a license to the top of.
#
# Only .h and .cpp are supported.
#
#  PARAMETERS:
#	1 - License file containing license text to add to files.
#
#  DEPENDENCIES:
#	
#  AUTHOR:
#	Eric Rock
#  CREATED:
#	9 May 2013
#  ENVIRONMENT:
#	(bash 4.2.24 on Linux 3.2.0-39)

usage(){
	echo "Usage: $1 <LICENSE FILE>"
	exit 1
}

#Argument 1: License file
#Argument 2: Source Directory
#Argument 3: Filetype of source files in directory
insertLicenses(){
	if [[ -e tmp ]]; then
		rm tmp
	fi
	
	for files in `pwd`/"$2"/*."$3"; do
		echo '/*' > tmp
		cat "$1" >> tmp
		echo '*/' >> tmp
		cat "$files" >> tmp
		rm "$files"
		mv tmp "$files"
	done
}

if (( $# < 1 )); then
	usage $0
fi

if [[ ! -e "$1" ]]; then
	echo "Could not find license file specified."
	exit 2
fi

if [[ -d src ]]; then
	insertLicenses "$1" src cpp
fi
if [[ -d include ]]; then
	insertLicenses "$1" include h
fi

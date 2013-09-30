/*
 * Trajectory.cpp
 *
 *  Created on: Sep 23, 2013
 *      Author: maestro
 */

#include "Trajectory.h"

const char Trajectory::DELIMITER = '\t';

Trajectory::Trajectory(int bufferSize){
	col2name = new vector<string>(40);

	(*col2name)[0] = "RHY";
	(*col2name)[1] = "RHR";
	(*col2name)[2] = "RHP";
	(*col2name)[3] = "RKN";
	(*col2name)[4] = "RAP";
	(*col2name)[5] = "RAR";
	(*col2name)[6] = "LHY";
	(*col2name)[7] = "LHR";
	(*col2name)[8] = "LHP";
	(*col2name)[9] = "LKN";
	(*col2name)[10] = "LAP";
	(*col2name)[11] = "LAR";
	(*col2name)[12] = "RSP";
	(*col2name)[13] = "RSR";
	(*col2name)[14] = "RSY";
	(*col2name)[15] = "REB";
	(*col2name)[16] = "RWY";
	(*col2name)[17] = "RWR";
	(*col2name)[18] = "RWP";
	(*col2name)[19] = "LSP";
	(*col2name)[20] = "LSR";
	(*col2name)[21] = "LSY";
	(*col2name)[22] = "LEB";
	(*col2name)[23] = "LWY";
	(*col2name)[24] = "LWR";
	(*col2name)[25] = "LWP";
	(*col2name)[26] = "NKY";
	(*col2name)[27] = "NK1";
	(*col2name)[28] = "NK2";
	(*col2name)[29] = "WST";
	(*col2name)[30] = "RF1";
	(*col2name)[31] = "RF2";
	(*col2name)[32] = "RF3";
	(*col2name)[33] = "RF4";
	(*col2name)[34] = "RF5";
	(*col2name)[35] = "LF1";
	(*col2name)[36] = "LF2";
	(*col2name)[37] = "LF3";
	(*col2name)[38] = "LF4";
	(*col2name)[39] = "LF5";

	buffers = new HashMap<string, vector<double>* >;
}

Trajectory::~Trajectory(){
	map<string, vector<double>* >::iterator i;
	for (i = buffers->begin(); i != buffers->end(); i++)
		delete (i->second);
	delete buffers;
}

void Trajectory::open(string path){

	std::cout << "Opening" << std::endl;
	trajInput.close();
	trajInput.open(path.c_str());
	if (!trajInput.is_open()) return; //TODO: Log error

	map<string, vector<double>* >::iterator it;
	for (it = buffers->begin(); it != buffers->end(); it++)
		delete it->second;
	std::cout << "after delete" << std::endl;
	for (int i = 0; i < col2name->size(); i++)
		(*buffers)[(*col2name)[i]] = new vector<double>(bufferSize);

	frames = 0;
	this->bufferSize = bufferSize;
	inputEnded = false;
	trajectoryEnded = false;

	reload();
	std::cout << "after reload" << std::endl;
}

bool Trajectory::is_open(){
	return trajInput.is_open() && !trajectoryEnded;
}

double Trajectory::nextPosition(string entry, double currentPosition){

	if (trajectoryEnded || buffers == NULL || buffers->count(entry) != 1 ||
				(*buffers)[entry] == NULL || (*buffers)[entry]->size() == 0) {

		return currentPosition;
	}

	double pos = (*(*buffers)[entry])[frames++];

	if (frames == bufferSize)
		reload();

	return pos;
}

void Trajectory::reload(){
	if (inputEnded || trajectoryEnded) return;

	frames = 0;

	string line;
	stringstream joints;
	string entry;
	inputEnded = false;

	for (int i = 0; i < bufferSize; i++){
		line = "";
		entry = "";

		// Grab one line of (hopefully) 40 columns of input positions
		getline(trajInput, line, '\n');
		// Convert that line into another stream for further parsing
		joints.str(line + DELIMITER);
		// The newline was consumed by the getline operation, so we will replace it with the given delimiter
		//joints << DELIMITER;

		for (int col = 0; col < col2name->size(); col++){
			// Grab an individual entry to parse, assuming entries are delimited by tabs
			getline(joints, entry, DELIMITER);
			// If we still think there's data to grab, store the value into the buffer vector mapped to the name of the joint of the current column.
			if (!inputEnded && sscanf(entry.c_str(), "%lf", &((*buffers)[(*col2name)[col]][i])) != 1){
				// The line is garbage. What to do?
				if (i == 0) {
					trajectoryEnded = true; // There is no more valid output to be had. Trajectory is finished.
					return;
				} else
					inputEnded = true; // There is still valid output left, but not enough to fill the buffer completely.
			}

			// If the buffer cannot be fully filled, pad the end of the buffer with the last valid data entry.
			if (inputEnded)
				(*buffers)[(*col2name)[col]][i] = (*buffers)[(*col2name)[col]][i - 1];
		}
	}
}

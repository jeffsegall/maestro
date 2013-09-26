/*
 * Trajectory.h
 *
 *  Created on: Sep 23, 2013
 *      Author: maestro
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <sstream>

using namespace std;

class Trajectory {

public:
	Trajectory();
	Trajectory(int bufferSize);
	~Trajectory();

	void open(string path);
	bool is_open();
	double nextPosition(string entry, double currentPosition);

private:

	void reload();


private:

	vector<string> *col2name;
	const char DELIMITER;
	ifstream trajInput;
	map<string, vector<double>* > *buffers;

	int frames;
	int bufferSize;
	bool inputEnded, trajectoryEnded;

};


#endif /* TRAJECTORY_H_ */

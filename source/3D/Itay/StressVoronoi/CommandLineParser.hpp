//\file CommandLineParser.hpp
//\brief A small class that handles the command line
//\author Itay Zandbank
#ifndef COMMANDLINEPARSER_HPP
#define COMMANDLINEPARSER_HPP

#include <string>

struct Arguments
{
	int NumPoints;
	std::string OutputDirectory;
	std::string InputFile;
	bool RunBruteForce;
	bool RunFullBruteForce;
	bool RunRigidWall;
	bool RunPeriodic;
};


bool ParseArguments(int argc, char *argv[], Arguments &args);

#endif
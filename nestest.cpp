#include "Cpu.h"
#include <iostream>
#include <fstream>
#include <vector>

// init hardware (outside so it's on heap)
Bus bus;
Cpu cpu(bus);

size_t PROGRAM_SIZE = 16 * 1024;

std::vector<uint8_t> readNesProgram(const std::string& fpath)
{

	std::ifstream file(fpath, std::ios::binary);

	if (!file.is_open())
	{
		std::cerr << "Could not open file at " << fpath << std::endl;
		exit(1);
	}
	// ines files have a 16bit header 
	// that doens't have the instructions to test for the log
	// skipping 16 positions from the beginning
	file.seekg(16, std::ios_base::beg);
	
	// create a vector containing 16kb
	std::vector<uint8_t> program(PROGRAM_SIZE);

	// read the bytes from the file for the full program size
	file.read((char*)program.data(), PROGRAM_SIZE);
	file.close();
	return program;

}


int main(int argc, char* argv[]) 
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << std::endl;
        return 1;
    }
    std::string filePath(argv[1]);

    cpu.initInstructions();
    cpu.reset(); 

    std::vector<uint8_t> program = readNesProgram(
        filePath
    );

    bus.loadProgram(program, 0x8000);
    bus.loadProgram(program, 0xC000);

    // nestest auto-mode starts at 0xC000.
    cpu.setPC(0xC000);

    for (int step = 0; step < 26554; step++)
    {

		std::string instructionStr = cpu.disassembleInstruction(cpu.getPC());

        std::cout << std::left << std::setw(48) << instructionStr
            << cpu.cpuState() << std::endl;

		cpu.cycle();
        
    }

    return 0;
}